// Gazebo world plugin: the hand tool of the simulation, and the balls it throws.
//
// At start-up every object of '/testing/objects' is spawned as a ball resting at its
// 'sim_position' (centre, robot base frame), on a thin static shelf built under them.
// Then it listens to the topics the arduino of the real tool listens to, so the testing
// node runs unchanged:
//   arduino/suctioning   the resting ball closest to the tube tip is attached: its centre
//                        is held on the tip, with no contacts, until the release
//   arduino/blowing_off  it is released: tip velocity + sqrt(2 E / m) along the tube x,
//                        E being the measured 'energy' of the object
// When a ball comes down to the floor its landing point is logged in the robot base
// frame, next to its target. The floor, the shelf, the balls and the landing points are
// published as rviz markers on 'ball_throw/markers' (visualization only).
//
// Everything runs in the physics thread: the ROS callbacks are served from onUpdate(),
// so the hold and the release happen on a physics step.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/MarkerArray.h>
#include "handtool_throw/yaml_helpers.h"

namespace gazebo
{

using ignition::math::Pose3d;
using ignition::math::Quaterniond;
using ignition::math::Vector3d;

// --- one ball, from an entry of '/testing/objects' --- //
struct Ball {
	enum State { RESTING, HELD, FLYING, LANDED };
	std::string name;
	double mass = 0.0;					// [kg]
	double energy = 0.0;				// [J] launch energy
	double radius = 0.0;				// [m]
	Vector3d rest;						// [m] centre where it is spawned, robot base frame
	bool has_target = false;
	Vector3d target;					// [m] robot base frame
	State state = RESTING;
	physics::ModelPtr model;			// null until the spawned ball exists
	Vector3d release;					// [m] world, where it left the tip
	bool collides = true;				// contacts on (off while held and just after the release)
	Vector3d landing;					// [m] robot base frame
};

// --- reads the balls of '/testing/objects': mass, energy, radius and sim_position are required --- //
static bool readBalls(std::vector<Ball> &balls)
{
	using namespace handtool_throw;
	XmlRpc::XmlRpcValue list;
	if (!ros::param::get("/testing/objects", list) || (list.getType() != XmlRpc::XmlRpcValue::TypeArray)){
		ROS_ERROR("ball_throw: no '/testing/objects' list, load the sim objects yaml");
		return false;
	}
	for (int i = 0; i < list.size(); i++){
		XmlRpc::XmlRpcValue &item = list[i];
		Ball ball;
		Eigen::Vector3d v;
		ball.name = "ball_" + std::to_string(i);
		if (item.getType() != XmlRpc::XmlRpcValue::TypeStruct){
			ROS_ERROR("ball_throw: objects[%d] is not a dictionary, skipped", i);
			continue;
		}
		if (item.hasMember("name") && (item["name"].getType() == XmlRpc::XmlRpcValue::TypeString)){
			ball.name = static_cast<std::string>(item["name"]);
		}
		if ((!item.hasMember("mass")) || (!getDouble(item["mass"], ball.mass)) || (ball.mass <= 0.0)
			|| (!item.hasMember("energy")) || (!getDouble(item["energy"], ball.energy)) || (ball.energy <= 0.0)
			|| (!item.hasMember("radius")) || (!getDouble(item["radius"], ball.radius)) || (ball.radius <= 0.0)
			|| (!item.hasMember("sim_position")) || (!getVector3(item["sim_position"], v))){
			ROS_ERROR("ball_throw: objects[%d] (%s) needs positive 'mass', 'energy', 'radius' and a "
				"'sim_position', skipped", i, ball.name.c_str());
			continue;
		}
		ball.rest = Vector3d(v(0), v(1), v(2));
		if (item.hasMember("target") && getVector3(item["target"], v)){
			ball.has_target = true;
			ball.target = Vector3d(v(0), v(1), v(2));
		}
		balls.push_back(ball);
	}
	return !balls.empty();
}

// --- solid sphere --- //
static std::string ballSdf(const std::string &name, const Ball &ball, const Vector3d &pos)
{
	const double r = ball.radius;
	const double I = 0.4 * ball.mass * r * r;
	std::ostringstream s;
	s << "<sdf version='1.6'><model name='" << name << "'>"
	  << "<pose>" << pos.X() << " " << pos.Y() << " " << pos.Z() << " 0 0 0</pose>"
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

// --- static box --- //
static std::string boxSdf(const std::string &name, const Pose3d &pose, const Vector3d &size)
{
	std::ostringstream s;
	s << "<sdf version='1.6'><model name='" << name << "'><static>true</static>"
	  << "<pose>" << pose << "</pose><link name='link'>"
	  << "<collision name='collision'><geometry><box><size>" << size << "</size></box></geometry></collision>"
	  << "<visual name='visual'><geometry><box><size>" << size << "</size></box></geometry>"
	  << "<material><script><uri>file://media/materials/scripts/gazebo.material</uri>"
	  << "<name>Gazebo/Wood</name></script></material></visual>"
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
		tip_offset_ = sdf->Get<Vector3d>("tip_offset", Vector3d(0.14, 0.0, 0.127)).first;
		floor_z_ = sdf->Get<double>("floor_z", 0.0).first;
		grasp_dist_ = sdf->Get<double>("grasp_dist", 0.1).first;
		marker_period_ = 1.0 / sdf->Get<double>("marker_rate", 30.0).first;

		nh_.reset(new ros::NodeHandle());
		nh_->setCallbackQueue(&queue_);
		sub_suct_ = nh_->subscribe("arduino/suctioning", 1, &BallThrowPlugin::onSuction, this);
		sub_blow_ = nh_->subscribe("arduino/blowing_off", 1, &BallThrowPlugin::onBlow, this);
		pub_markers_ = nh_->advertise<visualization_msgs::MarkerArray>("ball_throw/markers", 1);
		update_ = event::Events::ConnectWorldUpdateBegin(std::bind(&BallThrowPlugin::onUpdate, this));
		ROS_INFO("ball_throw: tube tip = %s::%s + [%.3f, %.3f, %.3f] m, floor at z = %.3f m",
			robot_model_.c_str(), tip_link_.c_str(), tip_offset_.X(), tip_offset_.Y(), tip_offset_.Z(), floor_z_);
	}

private:
	// --- robot base and tube tip, world frame. The tube x axis is the x axis of the tip link --- //
	bool basePose(Pose3d &base)
	{
		physics::ModelPtr robot = world_->ModelByName(robot_model_);
		physics::LinkPtr link = robot ? robot->GetLink(base_link_) : nullptr;
		if (!link) return false;
		base = link->WorldPose();
		return true;
	}

	bool tipState(Pose3d &pose, Vector3d &vel)
	{
		physics::ModelPtr robot = world_->ModelByName(robot_model_);
		physics::LinkPtr link = robot ? robot->GetLink(tip_link_) : nullptr;
		if (!link){
			ROS_ERROR_THROTTLE(1.0, "ball_throw: no link %s::%s", robot_model_.c_str(), tip_link_.c_str());
			return false;
		}
		const Pose3d link_pose = link->WorldPose();
		pose = Pose3d(link_pose.Pos() + link_pose.Rot().RotateVector(tip_offset_), link_pose.Rot());
		vel = link->WorldLinearVel(tip_offset_);
		return true;
	}

	// --- the balls in their row and the shelf under them, once the robot exists --- //
	void spawnRow(const Pose3d &base)
	{
		row_done_ = true;
		if (!readBalls(balls_)) return;

		// - shelf: top at the lowest ball bottom, footprint of the row plus a margin - //
		const double margin = 0.05, thickness = 0.02;
		Vector3d lo(1e9, 1e9, 1e9), hi(-1e9, -1e9, -1e9);
		for (const Ball &b : balls_){
			lo = Vector3d(std::min(lo.X(), b.rest.X() - b.radius), std::min(lo.Y(), b.rest.Y() - b.radius),
						  std::min(lo.Z(), b.rest.Z() - b.radius));
			hi = Vector3d(std::max(hi.X(), b.rest.X() + b.radius), std::max(hi.Y(), b.rest.Y() + b.radius), 0.0);
		}
		shelf_size_ = Vector3d(hi.X() - lo.X() + 2 * margin, hi.Y() - lo.Y() + 2 * margin, thickness);
		shelf_centre_ = Vector3d(0.5 * (lo.X() + hi.X()), 0.5 * (lo.Y() + hi.Y()), lo.Z() - 0.5 * thickness);
		world_->InsertModelString(boxSdf("ball_shelf",
			Pose3d(base.Pos() + base.Rot().RotateVector(shelf_centre_), base.Rot()), shelf_size_));

		for (Ball &b : balls_){
			world_->InsertModelString(ballSdf(b.name, b, base.Pos() + base.Rot().RotateVector(b.rest)));
		}
		ROS_INFO("ball_throw: %zu balls spawned in their row", balls_.size());
	}

	void onSuction(const std_msgs::Empty::ConstPtr &)
	{
		Pose3d tip;
		Vector3d tip_vel;
		if (!tipState(tip, tip_vel)) return;
		// - the resting ball closest to the tip is the one under the suction cup - //
		Ball *closest = nullptr;
		double dist = std::numeric_limits<double>::infinity();
		for (Ball &b : balls_){
			if ((b.state != Ball::RESTING) || (!b.model)) continue;
			const double d = b.model->WorldPose().Pos().Distance(tip.Pos());
			if (d < dist){
				dist = d;
				closest = &b;
			}
		}
		if ((!closest) || (dist > grasp_dist_)){
			ROS_WARN("ball_throw: suction with no ball within %.3f m of the tip (closest %.3f m), nothing grabbed",
				grasp_dist_, dist);
			return;
		}
		closest->state = Ball::HELD;
		closest->collides = false;
		closest->model->SetCollideMode("none");
		ROS_INFO("ball_throw: %s on the tool (%.3f kg, %.3f J, r %.3f m)",
			closest->name.c_str(), closest->mass, closest->energy, closest->radius);
	}

	void onBlow(const std_msgs::Empty::ConstPtr &)
	{
		Ball *held = nullptr;
		for (Ball &b : balls_){
			if (b.state == Ball::HELD) held = &b;
		}
		Pose3d tip;
		Vector3d tip_vel;
		if ((!held) || (!tipState(tip, tip_vel))){
			ROS_WARN("ball_throw: blow with no ball on the tool, nothing is thrown");
			return;
		}
		const double v = std::sqrt(2.0 * held->energy / held->mass);
		const Vector3d axis = tip.Rot().RotateVector(Vector3d::UnitX);
		held->model->SetWorldPose(Pose3d(tip.Pos(), Quaterniond::Identity));
		held->model->SetLinearVel(tip_vel + v * axis);
		held->model->SetAngularVel(Vector3d::Zero);
		held->release = tip.Pos();
		held->state = Ball::FLYING;
		ROS_INFO("ball_throw: %s released at %.3f m/s, tube axis [%.3f, %.3f, %.3f] (pitch %.2f deg)",
			held->name.c_str(), v, axis.X(), axis.Y(), axis.Z(),
			std::asin(std::max(-1.0, std::min(1.0, axis.Z()))) * 180.0 / M_PI);
	}

	void onUpdate()
	{
		queue_.callAvailable();

		Pose3d base;
		if (!basePose(base)) return;	// the robot is not spawned yet
		if (!row_done_) spawnRow(base);

		Pose3d tip;
		Vector3d tip_vel;
		const bool has_tip = tipState(tip, tip_vel);
		for (Ball &b : balls_){
			// - the spawned models appear a few steps after the request - //
			if (!b.model){
				b.model = world_->ModelByName(b.name);
				continue;
			}
			// - on the tool: the ball follows the tip, its own dynamics is overwritten every step - //
			if ((b.state == Ball::HELD) && has_tip){
				b.model->SetWorldPose(Pose3d(tip.Pos(), Quaterniond::Identity));
				b.model->SetLinearVel(tip_vel);
				b.model->SetAngularVel(Vector3d::Zero);
			}
			if (b.state == Ball::FLYING){
				const Vector3d p = b.model->WorldPose().Pos();
				// - contacts back on once the ball is clear of the tube it was overlapping - //
				if ((!b.collides) && (p.Distance(b.release) > b.radius + 0.02)){
					b.model->SetCollideMode("all");
					b.collides = true;
				}
				// - first touch of the floor: where, in the robot base frame - //
				if (p.Z() <= floor_z_ + b.radius + 0.002){
					b.landing = base.Rot().RotateVectorReverse(p - base.Pos());
					b.state = Ball::LANDED;
					logLanding(b);
				}
			}
		}

		const double now = world_->SimTime().Double();
		if (now - last_markers_ >= marker_period_){
			last_markers_ = now;
			publishMarkers(base);
		}
	}

	// ----- diagnostics and visualization, nothing of the simulation depends on them ----- //
	void logLanding(const Ball &b)
	{
		if (b.has_target){
			ROS_INFO("ball_throw: %s landed at [%.3f, %.3f] m, target [%.3f, %.3f] m, miss %.3f m (base frame)",
				b.name.c_str(), b.landing.X(), b.landing.Y(), b.target.X(), b.target.Y(),
				std::hypot(b.landing.X() - b.target.X(), b.landing.Y() - b.target.Y()));
		}else{
			ROS_INFO("ball_throw: %s landed at [%.3f, %.3f] m (base frame)", b.name.c_str(), b.landing.X(), b.landing.Y());
		}
	}

	static visualization_msgs::Marker marker(const std::string &ns, int id, int type, const Vector3d &pos,
											 const Vector3d &scale, float r, float g, float b, float a)
	{
		visualization_msgs::Marker m;
		m.header.frame_id = "panda_link0";	// robot base frame, the one of the targets
		m.header.stamp = ros::Time::now();
		m.ns = ns;
		m.id = id;
		m.type = type;
		m.action = visualization_msgs::Marker::ADD;
		m.pose.position.x = pos.X();
		m.pose.position.y = pos.Y();
		m.pose.position.z = pos.Z();
		m.pose.orientation.w = 1.0;
		m.scale.x = scale.X();
		m.scale.y = scale.Y();
		m.scale.z = scale.Z();
		m.color.r = r;
		m.color.g = g;
		m.color.b = b;
		m.color.a = a;
		return m;
	}

	void publishMarkers(const Pose3d &base)
	{
		using visualization_msgs::Marker;
		visualization_msgs::MarkerArray msg;
		// - floor under the throwing area, the targets are drawn on it - //
		const double floor_base_z = base.Rot().RotateVectorReverse(Vector3d(0, 0, floor_z_) - base.Pos()).Z();
		msg.markers.push_back(marker("floor", 0, Marker::CUBE, Vector3d(3.0, 0.0, floor_base_z - 0.001),
									 Vector3d(8.0, 4.0, 0.002), 0.5, 0.5, 0.5, 0.5));
		if (row_done_ && (!balls_.empty())){
			msg.markers.push_back(marker("shelf", 0, Marker::CUBE, shelf_centre_, shelf_size_, 0.6, 0.4, 0.2, 1.0));
		}
		for (size_t i = 0; i < balls_.size(); i++){
			const Ball &b = balls_[i];
			if (!b.model) continue;
			const Vector3d p = base.Rot().RotateVectorReverse(b.model->WorldPose().Pos() - base.Pos());
			const double d = 2.0 * b.radius;
			msg.markers.push_back(marker("balls", i, Marker::SPHERE, p, Vector3d(d, d, d), 1.0, 0.5, 0.0, 1.0));
			if (b.state == Ball::LANDED){
				msg.markers.push_back(marker("landings", i, Marker::CYLINDER, b.landing,
											 Vector3d(0.06, 0.06, 0.006), 0.1, 0.4, 1.0, 0.9));
				Marker text = marker("landing_names", i, Marker::TEXT_VIEW_FACING, b.landing + Vector3d(0, 0, 0.1),
									 Vector3d(0, 0, 0.06), 0.1, 0.4, 1.0, 1.0);
				text.text = b.name;
				msg.markers.push_back(text);
			}
		}
		pub_markers_.publish(msg);
	}

	physics::WorldPtr world_;
	std::string robot_model_, tip_link_, base_link_;
	Vector3d tip_offset_;					// [m] tube tip in the tip link frame
	double floor_z_ = 0.0;					// [m] world height of the floor
	double grasp_dist_ = 0.1;				// [m] farthest ball centre the suction can grab
	double marker_period_ = 1.0 / 30.0;		// [s] sim time between two marker arrays
	double last_markers_ = -1e9;

	std::unique_ptr<ros::NodeHandle> nh_;
	ros::CallbackQueue queue_;
	ros::Subscriber sub_suct_, sub_blow_;
	ros::Publisher pub_markers_;
	event::ConnectionPtr update_;

	bool row_done_ = false;					// the row is spawned once, at the first robot pose
	std::vector<Ball> balls_;
	Vector3d shelf_centre_, shelf_size_;	// [m] robot base frame
};

GZ_REGISTER_WORLD_PLUGIN(BallThrowPlugin)

}  // namespace gazebo
