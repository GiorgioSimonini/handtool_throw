// Gazebo world plugin: the hand tool of the simulation.
//
// It listens to the same topics the arduino of the real tool listens to, so the
// testing node runs unchanged:
//   arduino/suctioning   spawns the next ball of '/testing/objects' and holds it
//                        on the tube tip (centre on the tip, no contacts)
//   arduino/blowing_off  releases it: tip velocity + sqrt(2 E / m) along the tube x,
//                        E being the measured 'energy' of the object
// The balls are taken from '/testing/objects' in order, one per suction: the
// sequence of choice 7 grabs them in the same order. When the ball comes down to
// the floor its landing point is logged in the robot base frame, next to its target.
//
// Everything runs in the physics thread: the ROS callbacks are served from
// onUpdate(), so the hold and the release happen on a physics step.

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace gazebo
{

// --- one ball, read from an entry of '/testing/objects' --- //
struct Ball {
	std::string name;
	double mass = 0.0;					// [kg]
	double energy = 0.0;				// [J] launch energy
	double radius = 0.0;				// [m]
	bool has_target = false;
	ignition::math::Vector3d target;	// [m] robot base frame
};

// yaml writes 3 as an int and 3.0 as a double, accept both
static bool getDouble(XmlRpc::XmlRpcValue &val, double &out)
{
	if (val.getType() == XmlRpc::XmlRpcValue::TypeDouble){
		out = static_cast<double>(val);
		return true;
	}
	if (val.getType() == XmlRpc::XmlRpcValue::TypeInt){
		out = static_cast<int>(val);
		return true;
	}
	return false;
}

// --- reads entry i of '/testing/objects': mass, energy and radius are required --- //
static bool readBall(int i, Ball &ball)
{
	XmlRpc::XmlRpcValue list;
	if (!ros::param::get("/testing/objects", list) || (list.getType() != XmlRpc::XmlRpcValue::TypeArray)
		|| (list.size() == 0)){
		ROS_ERROR("ball_throw: no '/testing/objects' list, load the sim objects yaml");
		return false;
	}
	XmlRpc::XmlRpcValue &item = list[i % list.size()];
	if (item.getType() != XmlRpc::XmlRpcValue::TypeStruct){
		ROS_ERROR("ball_throw: objects[%d] is not a dictionary", i % list.size());
		return false;
	}
	ball.name = "ball";
	if (item.hasMember("name") && (item["name"].getType() == XmlRpc::XmlRpcValue::TypeString)){
		ball.name = static_cast<std::string>(item["name"]);
	}
	if ((!item.hasMember("mass")) || (!getDouble(item["mass"], ball.mass)) || (ball.mass <= 0.0)
		|| (!item.hasMember("energy")) || (!getDouble(item["energy"], ball.energy)) || (ball.energy <= 0.0)
		|| (!item.hasMember("radius")) || (!getDouble(item["radius"], ball.radius)) || (ball.radius <= 0.0)){
		ROS_ERROR("ball_throw: objects[%d] (%s) needs positive 'mass', 'energy' and 'radius'",
			i % list.size(), ball.name.c_str());
		return false;
	}
	ball.has_target = false;
	if (item.hasMember("target") && (item["target"].getType() == XmlRpc::XmlRpcValue::TypeArray)
		&& (item["target"].size() == 3)){
		double t[3];
		ball.has_target = getDouble(item["target"][0], t[0]) && getDouble(item["target"][1], t[1])
						  && getDouble(item["target"][2], t[2]);
		ball.target = ignition::math::Vector3d(t[0], t[1], t[2]);
	}
	return true;
}

// --- solid sphere, spawned far from everything: it is moved on the tip once it exists --- //
static std::string ballSdf(const std::string &model_name, const Ball &ball)
{
	const double r = ball.radius;
	const double I = 0.4 * ball.mass * r * r;
	std::ostringstream s;
	s << "<sdf version='1.6'><model name='" << model_name << "'>"
	  << "<pose>0 0 20 0 0 0</pose>"
	  << "<link name='ball'>"
	  << "<inertial><mass>" << ball.mass << "</mass><inertia>"
	  << "<ixx>" << I << "</ixx><iyy>" << I << "</iyy><izz>" << I << "</izz>"
	  << "<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
	  << "<collision name='collision'><geometry><sphere><radius>" << r << "</radius></sphere></geometry></collision>"
	  << "<visual name='visual'><geometry><sphere><radius>" << r << "</radius></sphere></geometry>"
	  << "<material><script><uri>file://media/materials/scripts/gazebo.material</uri>"
	  << "<name>Gazebo/Orange</name></script></material></visual>"
	  << "</link></model></sdf>";
	return s.str();
}


class BallThrowPlugin : public WorldPlugin
{
public:
	void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override
	{
		if (!ros::isInitialized()){
			gzerr << "ball_throw: ROS is not initialized, start gazebo through gazebo_ros\n";
			return;
		}
		world_ = world;
		// - where the tube tip is: Gazebo merges the fixed tube into the last moving link - //
		robot_model_ = sdf->Get<std::string>("robot_model", "panda").first;
		tip_link_ = sdf->Get<std::string>("tip_link", "panda_link7").first;
		base_link_ = sdf->Get<std::string>("base_link", "panda_link0").first;
		tip_offset_ = sdf->Get<ignition::math::Vector3d>("tip_offset",
					  ignition::math::Vector3d(0.14, 0.0, 0.127)).first;
		floor_z_ = sdf->Get<double>("floor_z", 0.0).first;

		nh_.reset(new ros::NodeHandle());
		nh_->setCallbackQueue(&queue_);
		sub_suct_ = nh_->subscribe("arduino/suctioning", 1, &BallThrowPlugin::onSuction, this);
		sub_blow_ = nh_->subscribe("arduino/blowing_off", 1, &BallThrowPlugin::onBlow, this);
		update_ = event::Events::ConnectWorldUpdateBegin(std::bind(&BallThrowPlugin::onUpdate, this));
		ROS_INFO("ball_throw: tube tip = %s::%s + [%.3f, %.3f, %.3f] m, floor at z = %.3f m",
			robot_model_.c_str(), tip_link_.c_str(), tip_offset_.X(), tip_offset_.Y(), tip_offset_.Z(), floor_z_);
	}

private:
	// --- tube tip, world frame. The tube x axis is the x axis of the tip link --- //
	bool tipState(ignition::math::Pose3d &pose, ignition::math::Vector3d &vel)
	{
		physics::ModelPtr robot = world_->ModelByName(robot_model_);
		physics::LinkPtr link = robot ? robot->GetLink(tip_link_) : nullptr;
		if (!link){
			ROS_ERROR_THROTTLE(1.0, "ball_throw: no link %s::%s", robot_model_.c_str(), tip_link_.c_str());
			return false;
		}
		const ignition::math::Pose3d link_pose = link->WorldPose();
		pose = ignition::math::Pose3d(link_pose.Pos() + link_pose.Rot().RotateVector(tip_offset_), link_pose.Rot());
		vel = link->WorldLinearVel(tip_offset_);
		return true;
	}

	void onSuction(const std_msgs::Empty::ConstPtr &)
	{
		if (held_){
			ROS_WARN("ball_throw: suction while %s is still on the tool, it is removed", model_name_.c_str());
			world_->RemoveModel(model_name_);
		}
		if (!readBall(n_balls_, ball_)){
			held_ = false;
			return;
		}
		// unique name: choice 7 can be run again on the same object list
		model_name_ = ball_.name + "_" + std::to_string(n_balls_);
		n_balls_++;
		world_->InsertModelString(ballSdf(model_name_, ball_));
		ball_model_.reset();
		held_ = true;
		flying_ = false;
		ROS_INFO("ball_throw: %s on the tool (%.3f kg, %.3f J, r %.3f m)",
			model_name_.c_str(), ball_.mass, ball_.energy, ball_.radius);
	}

	void onBlow(const std_msgs::Empty::ConstPtr &)
	{
		ignition::math::Pose3d tip;
		ignition::math::Vector3d tip_vel;
		if ((!held_) || (!ball_model_) || (!tipState(tip, tip_vel))){
			ROS_WARN("ball_throw: blow with no ball on the tool, nothing is thrown");
			return;
		}
		const double v = std::sqrt(2.0 * ball_.energy / ball_.mass);
		const ignition::math::Vector3d axis = tip.Rot().RotateVector(ignition::math::Vector3d::UnitX);
		ball_model_->SetWorldPose(ignition::math::Pose3d(tip.Pos(), ignition::math::Quaterniond::Identity));
		ball_model_->SetLinearVel(tip_vel + v * axis);
		ball_model_->SetAngularVel(ignition::math::Vector3d::Zero);
		release_pos_ = tip.Pos();
		held_ = false;
		flying_ = true;
		collides_ = false;
		ROS_INFO("ball_throw: %s released at %.3f m/s, tube axis [%.3f, %.3f, %.3f] (pitch %.2f deg)",
			model_name_.c_str(), v, axis.X(), axis.Y(), axis.Z(),
			std::asin(std::max(-1.0, std::min(1.0, axis.Z()))) * 180.0 / M_PI);
	}

	void onUpdate()
	{
		queue_.callAvailable();

		// - the spawned ball appears a few steps after the request: no contacts from then on - //
		if (held_ && (!ball_model_)){
			ball_model_ = world_->ModelByName(model_name_);
			if (ball_model_){
				ball_model_->SetCollideMode("none");
			}
		}

		// - on the tool: the ball follows the tip, its own dynamics is overwritten every step - //
		ignition::math::Pose3d tip;
		ignition::math::Vector3d tip_vel;
		if (held_ && ball_model_ && tipState(tip, tip_vel)){
			ball_model_->SetWorldPose(ignition::math::Pose3d(tip.Pos(), ignition::math::Quaterniond::Identity));
			ball_model_->SetLinearVel(tip_vel);
			ball_model_->SetAngularVel(ignition::math::Vector3d::Zero);
		}

		if (flying_){
			const ignition::math::Vector3d p = ball_model_->WorldPose().Pos();
			// - contacts back on once the ball is clear of the tube it was overlapping - //
			if ((!collides_) && (p.Distance(release_pos_) > ball_.radius + 0.02)){
				ball_model_->SetCollideMode("all");
				collides_ = true;
			}
			// - first touch of the floor: log where, in the robot base frame - //
			if (p.Z() <= floor_z_ + ball_.radius + 0.002){
				logLanding(p);
				flying_ = false;
			}
		}
	}

	// diagnostics only, nothing of the simulation depends on it
	void logLanding(const ignition::math::Vector3d &p_world)
	{
		physics::ModelPtr robot = world_->ModelByName(robot_model_);
		physics::LinkPtr base = robot ? robot->GetLink(base_link_) : nullptr;
		if (!base){
			ROS_INFO("ball_throw: %s landed at [%.3f, %.3f] m (world)", model_name_.c_str(), p_world.X(), p_world.Y());
			return;
		}
		const ignition::math::Pose3d b = base->WorldPose();
		const ignition::math::Vector3d p = b.Rot().RotateVectorReverse(p_world - b.Pos());
		if (ball_.has_target){
			ROS_INFO("ball_throw: %s landed at [%.3f, %.3f] m, target [%.3f, %.3f] m, miss %.3f m (base frame)",
				model_name_.c_str(), p.X(), p.Y(), ball_.target.X(), ball_.target.Y(),
				std::hypot(p.X() - ball_.target.X(), p.Y() - ball_.target.Y()));
		}else{
			ROS_INFO("ball_throw: %s landed at [%.3f, %.3f] m (base frame)", model_name_.c_str(), p.X(), p.Y());
		}
	}

	physics::WorldPtr world_;
	std::string robot_model_, tip_link_, base_link_;
	ignition::math::Vector3d tip_offset_;	// [m] tube tip in the tip link frame
	double floor_z_ = 0.0;					// [m] world height of the floor

	std::unique_ptr<ros::NodeHandle> nh_;
	ros::CallbackQueue queue_;
	ros::Subscriber sub_suct_, sub_blow_;
	event::ConnectionPtr update_;

	int n_balls_ = 0;					// suctions served, index of the next object
	Ball ball_;							// the ball on the tool or in flight
	std::string model_name_;
	physics::ModelPtr ball_model_;		// null until the spawned ball exists
	bool held_ = false;					// on the tube tip
	bool flying_ = false;				// released, not yet on the floor
	bool collides_ = false;				// contacts enabled again after the release
	ignition::math::Vector3d release_pos_;
};

GZ_REGISTER_WORLD_PLUGIN(BallThrowPlugin)

}  // namespace gazebo
