#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <eigen3/Eigen/Dense>
#include <limits>
#include <string>
#include <vector>
#include <xmlrpcpp/XmlRpcValue.h>
#include "handtool_throw/throwing_par_srv.h"

// #include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include "panda_controllers/desTrajEE.h"
// #include <eigen_conversions/eigen_msg.h>

// #include<tf_conversions/tf_eigen.h>

using namespace std;
using Eigen::VectorXd;

// functions
VectorXd get_q_minj(double t, double tf, VectorXd q_i, VectorXd q_f, VectorXd dq_i, VectorXd dq_f, VectorXd ddq_i, VectorXd ddq_f){
		VectorXd q_t = q_i + dq_i*t + (ddq_i*pow(t,2))/2 - (pow(t,5)*(12*q_i - 12*q_f + 6*dq_f*tf + 6*dq_i*tf - ddq_f*pow(tf,2) + ddq_i*pow(tf,2)))/(2*pow(tf,5)) - (pow(t,3)*(20*q_i - 20*q_f + 8*dq_f*tf + 12*dq_i*tf - ddq_f*pow(tf,2) + 3*ddq_i*pow(tf,2)))/(2*pow(tf,3)) + (pow(t,4)*(30*q_i - 30*q_f + 14*dq_f*tf + 16*dq_i*tf - 2*ddq_f*pow(tf,2) + 3*ddq_i*pow(tf,2)))/(2*pow(tf,4));
		return q_t;
}
VectorXd get_dq_minj(double t, double tf, VectorXd q_i, VectorXd q_f, VectorXd dq_i, VectorXd dq_f, VectorXd ddq_i, VectorXd ddq_f){
		VectorXd dq_t = dq_i + ddq_i*t - (5*pow(t,4)*(12*q_i - 12*q_f + 6*dq_f*tf + 6*dq_i*tf - ddq_f*pow(tf,2) + ddq_i*pow(tf,2)))/(2*pow(tf,5)) - (3*pow(t,2)*(20*q_i - 20*q_f + 8*dq_f*tf + 12*dq_i*tf - ddq_f*pow(tf,2) + 3*ddq_i*pow(tf,2)))/(2*pow(tf,3)) + (2*pow(t,3)*(30*q_i - 30*q_f + 14*dq_f*tf + 16*dq_i*tf - 2*ddq_f*pow(tf,2) + 3*ddq_i*pow(tf,2)))/pow(tf,4);
		return dq_t;
}

VectorXd get_ddq_minj(double t, double tf, VectorXd q_i, VectorXd q_f, VectorXd dq_i, VectorXd dq_f, VectorXd ddq_i, VectorXd ddq_f){
		VectorXd ddq_t = ddq_i - (3*t*(20*q_i - 20*q_f + 8*dq_f*tf + 12*dq_i*tf - ddq_f*pow(tf,2) + 3*ddq_i*pow(tf,2)))/pow(tf,3) - (10*pow(t,3)*(12*q_i - 12*q_f + 6*dq_f*tf + 6*dq_i*tf - ddq_f*pow(tf,2) + ddq_i*pow(tf,2)))/pow(tf,5) + (6*pow(t,2)*(30*q_i - 30*q_f + 14*dq_f*tf + 16*dq_i*tf - 2*ddq_f*pow(tf,2) + 3*ddq_i*pow(tf,2)))/pow(tf,4);
		return ddq_t;
}

// Callback for franka base
void frankaBaseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
void targetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
void frankaCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

// int counter = 0;
std_msgs::Empty empty_msg;
std_msgs::UInt32 valve_msg;
std_msgs::UInt8 regulator_msg;
geometry_msgs::Point target;
Eigen::Affine3d franka_pose;	// measured robot pose
Eigen::Affine3d franka_pose_d;	// desired throwing pose, from the handtool service

Eigen::Vector3d franka_pos_qualisys;
Eigen::Quaterniond franka_quat_qualisys;

double pre_suct_time = 2.0;
double suct_time = 0.5;
int valve_us = 50000; 	// valve time opering in microseconds
int duty_regulator = 255; // 255: bypass regulator

bool has_frankaBase_pose = false;
bool has_franka_pose = false;
bool has_target_pose = false;

// --- object sequence (choice 7), all the poses are in the robot base frame --- //
Eigen::Affine3d default_pose;						// pose the sequence starts from and returns to
bool has_default_pose = false;						// false: the yaml does not carry a valid one
Eigen::Vector3d approach_offset(-0.1, 0.0, 0.0);	// [m] grasping pose -> approach pose, tip frame
double move_time = 3.0;								// [s] duration of the long motions
double approach_time = 1.5;							// [s] duration of the approach/retreat motions
double pre_throw_time = 1.0;						// [s] settling time on the throwing pose, before the throw

// --- one entry of the object list of the yaml --- //
struct Object {
	string name;					// only used in the printouts
	double m_obj;					// [kg] mass, sent to the throwing parameters service
	Eigen::Affine3d grasp_pose;		// pose the suction cup grabs the object from
	bool has_target;				// true: the yaml carries a target, qualisys is not needed
	geometry_msgs::Point target;	// [m] target box position

	Object() : m_obj(0.0), has_target(false) {
		grasp_pose.setIdentity();
	}
};


// ----- yaml helpers ----- //

// --- reads a scalar: yaml writes 3 as an int and 3.0 as a double, accept both --- //
bool getDouble(XmlRpc::XmlRpcValue &val, double &out)
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

// --- reads a [x, y, z] list --- //
bool getVector3(XmlRpc::XmlRpcValue &val, Eigen::Vector3d &out)
{
	if ((val.getType() != XmlRpc::XmlRpcValue::TypeArray) || (val.size() != 3)){
		return false;
	}
	for (int i = 0; i < 3; i++){
		double value;
		if (!getDouble(val[i], value)){
			return false;
		}
		out(i) = value;
	}
	return true;
}

// --- reads a pose: a 'position' list plus either a quaternion or rpy angles --- //
bool getPose(XmlRpc::XmlRpcValue &val, Eigen::Affine3d &out)
{
	if (val.getType() != XmlRpc::XmlRpcValue::TypeStruct){
		return false;
	}
	Eigen::Vector3d position;
	if ((!val.hasMember("position")) || (!getVector3(val["position"], position))){
		return false;
	}

	Eigen::Quaterniond quat;
	if (val.hasMember("orientation")){
		// - quaternion [x, y, z, w], the form the throwing service answers with - //
		XmlRpc::XmlRpcValue &quat_par = val["orientation"];
		if ((quat_par.getType() != XmlRpc::XmlRpcValue::TypeArray) || (quat_par.size() != 4)){
			return false;
		}
		double qx, qy, qz, qw;
		if ((!getDouble(quat_par[0], qx)) || (!getDouble(quat_par[1], qy))
			|| (!getDouble(quat_par[2], qz)) || (!getDouble(quat_par[3], qw))){
			return false;
		}
		quat = Eigen::Quaterniond(qw, qx, qy, qz);
	}else if (val.hasMember("rpy_deg")){
		// - extrinsic xyz angles, same convention as dummy_qualisys.py, easier to write by hand - //
		Eigen::Vector3d rpy;
		if (!getVector3(val["rpy_deg"], rpy)){
			return false;
		}
		rpy *= M_PI/180.0;
		quat = Eigen::Quaterniond(
			Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ())
			* Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()));
	}else{
		return false;
	}
	// a null quaternion cannot be normalised into a rotation
	if (quat.norm() < 1e-9){
		return false;
	}

	// a default constructed Affine3d is uninitialized, the last row would stay garbage
	out.setIdentity();
	out.translation() = position;
	out.linear() = quat.normalized().toRotationMatrix();
	return true;
}

// --- reads the object list, a malformed entry is skipped instead of killing the sequence --- //
bool loadObjects(ros::NodeHandle &nh, vector<Object> &objects)
{
	objects.clear();
	XmlRpc::XmlRpcValue list;
	if (!nh.getParam("/testing/objects", list)){
		ROS_ERROR("No '/testing/objects' on the parameter server, load the objects yaml");
		return false;
	}
	if (list.getType() != XmlRpc::XmlRpcValue::TypeArray){
		ROS_ERROR("'/testing/objects' must be a list of objects");
		return false;
	}

	for (int i = 0; i < list.size(); i++){
		XmlRpc::XmlRpcValue &item = list[i];
		Object obj;
		obj.name = "object_" + to_string(i);

		if (item.getType() != XmlRpc::XmlRpcValue::TypeStruct){
			ROS_ERROR("objects[%d]: not a dictionary, skipped", i);
			continue;
		}
		if (item.hasMember("name") && (item["name"].getType() == XmlRpc::XmlRpcValue::TypeString)){
			obj.name = static_cast<string>(item["name"]);
		}
		// - mass, required by the throwing parameters service - //
		if ((!item.hasMember("mass")) || (!getDouble(item["mass"], obj.m_obj)) || (obj.m_obj <= 0.0)){
			ROS_ERROR("objects[%d] (%s): 'mass' missing, malformed or not positive, skipped", i, obj.name.c_str());
			continue;
		}
		// - grasping pose, required - //
		if ((!item.hasMember("grasp_pose")) || (!getPose(item["grasp_pose"], obj.grasp_pose))){
			ROS_ERROR("objects[%d] (%s): 'grasp_pose' missing or malformed, skipped", i, obj.name.c_str());
			continue;
		}
		// - target, optional: without it the one measured by qualisys is used - //
		if (item.hasMember("target")){
			Eigen::Vector3d target_par;
			if (!getVector3(item["target"], target_par)){
				ROS_ERROR("objects[%d] (%s): 'target' malformed, skipped", i, obj.name.c_str());
				continue;
			}
			obj.target.x = target_par(0);
			obj.target.y = target_par(1);
			obj.target.z = target_par(2);
			obj.has_target = true;
		}
		objects.push_back(obj);
	}

	if (objects.empty()){
		ROS_ERROR("'/testing/objects' holds no usable object");
		return false;
	}
	ROS_INFO("loaded %d objects", (int)objects.size());
	return true;
}


// ----- motion and hand-tool helpers, shared by the menu entries ----- //

// --- waits for a fresh robot pose, and for the qualisys target if requested --- //
bool waitForPoses(bool need_target)
{
	has_franka_pose = false;
	if (need_target){
		// the target is rejected by its callback until the base pose is known, reset both
		has_frankaBase_pose = false;
		has_target_pose = false;
	}
	auto start_time = ros::Time::now();
	while (ros::ok() && ((!has_franka_pose) || (need_target && (!has_target_pose)))){
		ros::spinOnce();
		// the poses come at the controller rate, warn only if one is really not published
		if ((ros::Time::now() - start_time).toSec() > 1.0){
			ROS_WARN_THROTTLE(2.0, "waiting for poses: robot %s, target %s",
				has_franka_pose ? "ok" : "missing",
				(!need_target) ? "not needed" : (has_target_pose ? "ok" : "missing"));
		}
		ros::Duration(0.01).sleep();
	}
	return ros::ok();
}

// --- minimum jerk cartesian motion, from the measured pose to pose_d in T seconds --- //
bool moveToPose(ros::Publisher &pub_command, const Eigen::Affine3d &pose_d, double T)
{
	if (T <= 0.0){
		ROS_ERROR("motion duration must be positive, got %f s", T);
		return false;
	}
	// the motion starts from where the robot is now, ask for a fresh measure
	if (!waitForPoses(false)){
		return false;
	}
	// frozen copy: the callbacks are not served during the trajectory cycle
	const Eigen::Affine3d pose_start = franka_pose;

	// - declarations - //
	Eigen::VectorXd pos_start(3), pos_end(3), theta_start(1), theta_end(1);
	Eigen::VectorXd vel_start(3), vel_end(3), dtheta_start(1), dtheta_end(1);
	Eigen::VectorXd pos_t(3), vel_t(3), acc_t(3), theta_t(1), dtheta_t(1), ddtheta_t(1);
	Eigen::VectorXd ZERO_1 = Eigen::VectorXd::Zero(1);
	Eigen::VectorXd ZERO_3 = Eigen::VectorXd::Zero(3);
	Eigen::Quaterniond franka_quat;

	// - common parts - //
	// from the measured pose to the desired one
	pos_start << pose_start.translation();
	pos_end << pose_d.translation();
	Eigen::Matrix3d franka_rot = pose_start.linear().transpose() * pose_d.linear();

	// the orientation moves as a single rotation around a fixed axis of the starting frame
	Eigen::AngleAxisd franka_angleAxis(franka_rot);
	double franka_angle = franka_angleAxis.angle();
	Eigen::Vector3d franka_axis = franka_angleAxis.axis();
	theta_start << 0.0;
	theta_end << franka_angle;

	// rest to rest motion
	vel_start.setZero();
	vel_end.setZero();
	dtheta_start.setZero();
	dtheta_end.setZero();

	// - trajectory cycle - //
	panda_controllers::desTrajEE cmd;
	ros::Rate rate(500);
	auto start_time = ros::Time::now();

	double t = (ros::Time::now() - start_time).toSec();
	while ((ros::ok()) && (t <= T)) {
		// interpolation
		pos_t = get_q_minj(t, T, pos_start, pos_end, vel_start, vel_end, ZERO_3, ZERO_3);
		vel_t = get_dq_minj(t, T, pos_start, pos_end, vel_start, vel_end, ZERO_3, ZERO_3);
		acc_t = get_ddq_minj(t, T, pos_start, pos_end, vel_start, vel_end, ZERO_3, ZERO_3);
		theta_t = get_q_minj(t, T, theta_start, theta_end, dtheta_start, dtheta_end, ZERO_1, ZERO_1);
		dtheta_t = get_dq_minj(t, T, theta_start, theta_end, dtheta_start, dtheta_end, ZERO_1, ZERO_1);
		ddtheta_t = get_ddq_minj(t, T, theta_start, theta_end, dtheta_start, dtheta_end, ZERO_1, ZERO_1);

		franka_quat = Eigen::Quaterniond(pose_start.linear()) * Eigen::Quaterniond(Eigen::AngleAxisd(theta_t(0), franka_axis));

		// filling message
		cmd.header.stamp = ros::Time::now();
		cmd.position.x = pos_t(0);
		cmd.position.y = pos_t(1);
		cmd.position.z = pos_t(2);
		cmd.orientation.x = franka_quat.x();
		cmd.orientation.y = franka_quat.y();
		cmd.orientation.z = franka_quat.z();
		cmd.orientation.w = franka_quat.w();
		// velocities
		cmd.velocity.x = vel_t(0);
		cmd.velocity.y = vel_t(1);
		cmd.velocity.z = vel_t(2);
		cmd.ang_vel.x = franka_axis(0)*dtheta_t(0);
		cmd.ang_vel.y = franka_axis(1)*dtheta_t(0);
		cmd.ang_vel.z = franka_axis(2)*dtheta_t(0);
		// accelerations
		cmd.acceleration.x = acc_t(0);
		cmd.acceleration.y = acc_t(1);
		cmd.acceleration.z = acc_t(2);
		cmd.ang_acc.x = franka_axis(0)*ddtheta_t(0);
		cmd.ang_acc.y = franka_axis(1)*ddtheta_t(0);
		cmd.ang_acc.z = franka_axis(2)*ddtheta_t(0);

		// publish message
		pub_command.publish(cmd);
		rate.sleep();
		// update time
		t = (ros::Time::now() - start_time).toSec();
	}
	return ros::ok();
}

// --- asks the service for the valve time and the throwing pose of one throw --- //
bool getThrowingPar(ros::ServiceClient &handtool_client, double m_obj,
					const geometry_msgs::Point &target_req,
					int &valve_us_out, Eigen::Affine3d &pose_d_out)
{
	handtool_throw::throwing_par_srv srv;
	srv.request.m_obj = m_obj;
	srv.request.target = target_req;
	if (!handtool_client.call(srv)){
		ROS_ERROR("Failed to call throw parameters service");
		return false;
	}
	if (!srv.response.answer){
		ROS_ERROR("Throw parameters service could not compute the throwing parameters for this request");
		return false;
	}

	valve_us_out = srv.response.result_valve_us;
	geometry_msgs::Pose pose = srv.response.result_pose;

	// the service returns the DESIRED throwing pose
	pose_d_out.setIdentity();
	pose_d_out.translation() = Eigen::Vector3d(
		pose.position.x,
		pose.position.y,
		pose.position.z);

	Eigen::Quaterniond quat(
		pose.orientation.w,
		pose.orientation.x,
		pose.orientation.y,
		pose.orientation.z);
	pose_d_out.linear() = quat.normalized().toRotationMatrix();

	std::cout << "Valve time in us:" << valve_us_out << std::endl;
	std::cout << "Position:" << std::endl;
	std::cout << "  x: " << pose.position.x << std::endl;
	std::cout << "  y: " << pose.position.y << std::endl;
	std::cout << "  z: " << pose.position.z << std::endl;
	std::cout << "Orientation:" << std::endl;
	std::cout << "  x: " << pose.orientation.x << std::endl;
	std::cout << "  y: " << pose.orientation.y << std::endl;
	std::cout << "  z: " << pose.orientation.z << std::endl;
	std::cout << "  w: " << pose.orientation.w << std::endl;
	return true;
}

// --- grabs the object, the vacuum then holds it until the release --- //
void graspObject(ros::Publisher &pub_suct)
{
	// wait
	ros::Duration(pre_suct_time).sleep();
	// suctioning
	pub_suct.publish(empty_msg);
	// time the cup needs to hold the object
	ros::Duration(suct_time).sleep();
}

// --- releases the object: the blow frees the cup and throws --- //
void throwObject(ros::Publisher &pub_throw)
{
	// throwing
	pub_throw.publish(empty_msg);
	ros::Duration(0.1).sleep();
}

// --- blocks until the user presses enter, false on closed input --- //
bool waitKeyPress(const string &message)
{
	string line;
	cout << message << endl;
	getline(cin, line);
	return !cin.eof();
}


int main(int argc, char **argv)
{       
	ros::init(argc, argv, "testing");
	ros::NodeHandle nh_;

	// a default constructed Affine3d is uninitialized
	franka_pose.setIdentity();
	franka_pose_d.setIdentity();
	default_pose.setIdentity();

	// Subscribers
	ros::Subscriber sub_target = nh_.subscribe("/qualisys/box_target/pose", 1, &targetCallback);
	ros::Subscriber sub_franka_base = nh_.subscribe("/qualisys/mpc_franka/pose", 1, &frankaBaseCallback);
	ros::Subscriber sub_franka = nh_.subscribe("/backstepping_controller/franka_pose", 1, &frankaCallback);

	// Publishers
	ros::Publisher pub_throw = nh_.advertise<std_msgs::Empty>("arduino/blowing_off", 1); 
	ros::Publisher pub_suct = nh_.advertise<std_msgs::Empty>("arduino/suctioning", 1); 
	ros::Publisher pub_reg = nh_.advertise<std_msgs::UInt8>("arduino/duty_cycle", 1); 
	ros::Publisher pub_valve = nh_.advertise<std_msgs::UInt32>("arduino/valve", 1);
	ros::Publisher pub_command = nh_.advertise<panda_controllers::desTrajEE>("/backstepping_controller/command_cartesian", 1);
	// handtool service
	ros::ServiceClient handtool_client = nh_.serviceClient<handtool_throw::throwing_par_srv>("handtool_throw_service");
	// parameters from yaml
	if (!nh_.getParam("/testing/pre_suct_time", pre_suct_time)) {
		ROS_WARN("Failed to get param");
	}
	if (!nh_.getParam("/testing/suct_time", suct_time)) {
		ROS_WARN("Failed to get param");
	}
	if (!nh_.getParam("/testing/valve_us", valve_us)) {
		ROS_WARN("Failed to get param");
	}
	if (!nh_.getParam("/testing/duty_regulator", duty_regulator)) {
		ROS_WARN("Failed to get param");
	}
	// - object sequence parameters, the object list itself is read at every run of choice 7 - //
	XmlRpc::XmlRpcValue default_pose_par;
	if (!nh_.getParam("/testing/default_pose", default_pose_par)){
		ROS_WARN("Failed to get param '/testing/default_pose', choice 7 stays disabled");
	}else if (!getPose(default_pose_par, default_pose)){
		ROS_WARN("Malformed param '/testing/default_pose', choice 7 stays disabled");
	}else{
		has_default_pose = true;
	}
	vector<double> approach_offset_par;
	if (nh_.getParam("/testing/approach_offset", approach_offset_par) && (approach_offset_par.size() == 3)){
		approach_offset = Eigen::Vector3d(approach_offset_par[0], approach_offset_par[1], approach_offset_par[2]);
	}else{
		ROS_WARN("Failed to get param '/testing/approach_offset', keeping [%f, %f, %f] m",
			approach_offset(0), approach_offset(1), approach_offset(2));
	}
	if (!nh_.getParam("/testing/move_time", move_time)) {
		ROS_WARN("Failed to get param");
	}
	if (!nh_.getParam("/testing/approach_time", approach_time)) {
		ROS_WARN("Failed to get param");
	}
	if (!nh_.getParam("/testing/pre_throw_time", pre_throw_time)) {
		ROS_WARN("Failed to get param");
	}
	// wait
	ros::Duration(1.0).sleep();

	std::cout << "Welcome to the hand-tool testing"<< std::endl;
	int choice;
	while(ros::ok()){
		cout<<"choice:   (1: set times (pre-suct,suct),  2: set regulator,  3: set valve time (us),  4: suck&throw,  5: get throw par,  6: throw loop,  7: objects sequence) "<<endl;
		cin>>choice;
		// stop on closed input, do not spin on a bad one
		if (cin.eof()){
			cout<<"input closed, exiting"<<endl;
			break;
		}
		if (cin.fail()){
			cin.clear();
			cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			ROS_WARN("Invalid choice, expected a number");
			continue;
		}
		// while (!ready) ros::spinOnce();
		if (choice == 1){
			// --- set times --- //
			cin >> pre_suct_time;
			cin >> suct_time;
			ros::Duration(0.1).sleep();
		}else if (choice == 2){
			// --- set duty-cycle of regulator --- //
			cin >> duty_regulator;
			regulator_msg.data = duty_regulator;
			pub_reg.publish(regulator_msg);
			ros::Duration(0.1).sleep();
		}else if (choice == 3){
			// --- set valve time --- //
			cin >> valve_us;
			valve_msg.data = valve_us;
			pub_valve.publish(valve_msg);
			ros::Duration(0.1).sleep();
		}else if (choice == 4){
			// --- suck and throw cycle --- //
			graspObject(pub_suct);
			throwObject(pub_throw);
		}else if (choice == 5){
			// --- get parameters for handtool throwing --- //
			cout << "insert object weight and target x, y, z positions (m_obj x y z):" << endl;
			double m_obj;
			geometry_msgs::Point target_req;
			cin >> m_obj;
			cin >> target_req.x;
			cin >> target_req.y;
			cin >> target_req.z;
			// the solution is printed by the helper, nothing else to do here
			int valve_opt;
			Eigen::Affine3d pose_opt;
			getThrowingPar(handtool_client, m_obj, target_req, valve_opt, pose_opt);
			ros::Duration(0.1).sleep();
		}else if (choice == 6){
			// --- Complete throwing cycle --- //
			// - get mass of object - //
			cout<<"insert object weight (m_obj): "<<endl;
			double m_obj;
			cin >> m_obj;

			// - get target box position and current robot pose - //
			cout<<"waiting for target and robot poses..."<<endl;
			if (!waitForPoses(true)) break;

			// - get throwing parameters - //
			if (!getThrowingPar(handtool_client, m_obj, target, valve_us, franka_pose_d)){
				ROS_ERROR("no throwing parameters, no command is sent");
				continue;
			}
			// - set valve optimal time - //
			valve_msg.data = valve_us;
			pub_valve.publish(valve_msg);

			// --- go to throwing pose --- //
			if (!moveToPose(pub_command, franka_pose_d, move_time)) break;

			// - throw - //
			graspObject(pub_suct);
			throwObject(pub_throw);
		}else if (choice == 7){
			// --- automatic sequence over the objects listed in the yaml --- //
			// for every object: default pose -> approach -> grasp -> suck -> retreat
			//                   -> throwing pose -> throw -> default pose -> key press
			if (!has_default_pose){
				ROS_ERROR("no valid '/testing/default_pose', fill it in the objects yaml first");
				continue;
			}
			// re-read the list at every run, the yaml can be reloaded with rosparam meanwhile
			vector<Object> objects;
			if (!loadObjects(nh_, objects)) continue;

			// drop the newline left by 'cin >> choice', the first key press would eat it
			cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

			for (size_t i = 0; (i < objects.size()) && ros::ok(); i++){
				const Object &obj = objects[i];
				cout<<"--- object "<<(i+1)<<"/"<<objects.size()<<": "<<obj.name
					<<" (mass "<<obj.m_obj<<" kg) ---"<<endl;

				// - target: the one of the yaml, or the measured one if the yaml has none - //
				geometry_msgs::Point target_obj;
				if (obj.has_target){
					target_obj = obj.target;
				}else{
					cout<<"no target in the yaml, waiting for the qualisys poses..."<<endl;
					if (!waitForPoses(true)) break;
					target_obj = target;	// filled by targetCallback, robot base frame
				}
				cout<<"target: ["<<target_obj.x<<", "<<target_obj.y<<", "<<target_obj.z<<"] m"<<endl;

				// - throwing parameters first: a failure must not leave the object grabbed - //
				int valve_obj;
				Eigen::Affine3d throw_pose;
				if (!getThrowingPar(handtool_client, obj.m_obj, target_obj, valve_obj, throw_pose)){
					ROS_ERROR("%s: no throwing parameters, object skipped", obj.name.c_str());
					continue;
				}
				// - set valve optimal time - //
				valve_us = valve_obj;
				valve_msg.data = valve_us;
				pub_valve.publish(valve_msg);

				// - the object is approached and left along approach_offset, expressed in the
				//   TIP frame of the grasping pose: -x moves the tool away from the object - //
				Eigen::Affine3d approach_pose = obj.grasp_pose * Eigen::Translation3d(approach_offset);

				// - go to the default position - //
				cout<<"going to the default position..."<<endl;
				if (!moveToPose(pub_command, default_pose, move_time)) break;

				// - reach the object, through the approach pose - //
				cout<<"reaching the object..."<<endl;
				if (!moveToPose(pub_command, approach_pose, move_time)) break;
				if (!moveToPose(pub_command, obj.grasp_pose, approach_time)) break;

				// - grab it, the vacuum holds until the throw - //
				cout<<"grabbing the object..."<<endl;
				graspObject(pub_suct);

				// - leave the grasping place before moving away - //
				if (!moveToPose(pub_command, approach_pose, approach_time)) break;

				// - pass through the default position: a single minimum jerk segment from
				//   the grasping side to the throwing pose interpolates only the cartesian
				//   pose, the arm is free to travel through odd configurations on the way.
				//   The default pose is a known good waypoint between the two - //
				cout<<"going back to the default position..."<<endl;
				if (!moveToPose(pub_command, default_pose, move_time)) break;

				// - move to the throwing pose - //
				cout<<"going to the throwing pose..."<<endl;
				if (!moveToPose(pub_command, throw_pose, move_time)) break;

				// - the controller reaches the commanded pose with some delay, give it
				//   the time to settle before letting the object go - //
				ros::Duration(pre_throw_time).sleep();

				// - throw - //
				cout<<"throwing..."<<endl;
				throwObject(pub_throw);

				// - back to the default position - //
				cout<<"going back to the default position..."<<endl;
				if (!moveToPose(pub_command, default_pose, move_time)) break;

				// - the next object is started by hand - //
				if (!waitKeyPress("object thrown, press enter to continue")) break;
			}
			cout<<"objects sequence ended"<<endl;
		}
	}
}

// Callback for franka base
void frankaBaseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	franka_pos_qualisys = Eigen::Vector3d(
		msg->pose.position.x,
		msg->pose.position.y,
		msg->pose.position.z);
		
	franka_quat_qualisys = Eigen::Quaterniond(
		msg->pose.orientation.w,
		msg->pose.orientation.x,
		msg->pose.orientation.y,
		msg->pose.orientation.z);
		
	has_frankaBase_pose = true;
}

// callback for target box
void targetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	if (!has_frankaBase_pose)
	{
		ROS_WARN_THROTTLE(1.0, "Qualisys: robot pose not yet received");
		return;
	}

	// Posizione ostacolo nel sistema Qualisys
	Eigen::Vector3d p_target_qualisys(
		msg->pose.position.x,
		msg->pose.position.y,
		msg->pose.position.z);

	// TRASFORMAZIONE: posizione ostacolo rispetto al centro del robot
	Eigen::Vector3d p_rel = franka_quat_qualisys.inverse() *
							(p_target_qualisys - franka_pos_qualisys);

	target.x = p_rel(0);
	target.y = p_rel(1);
	target.z = p_rel(2);

	has_target_pose = true;

}

// Callback for franka robot
void frankaCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	franka_pose.translation() = Eigen::Vector3d(
		msg->pose.position.x,
		msg->pose.position.y,
		msg->pose.position.z);
		
	Eigen::Quaterniond quat(
		msg->pose.orientation.w,
		msg->pose.orientation.x,
		msg->pose.orientation.y,
		msg->pose.orientation.z);
	franka_pose.linear() = quat.toRotationMatrix();
		
	has_franka_pose = true;
}
