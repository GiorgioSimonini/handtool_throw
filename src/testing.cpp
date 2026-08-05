#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <eigen3/Eigen/Dense>
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
Eigen::Affine3d franka_pose;
Eigen::Affine3d franka_pose_d;

Eigen::Vector3d franka_pos_qualisys;
Eigen::Quaterniond franka_quat_qualisys;

double pre_suct_time = 2.0;
double suct_time = 0.5;
int valve_us = 50000; 	// valve time opering in microseconds
int duty_regulator = 255; // 255: bypass regulator

bool has_frankaBase_pose = false;
bool has_franka_pose = false;
bool has_target_pose = false;


int main(int argc, char **argv)
{       
	ros::init(argc, argv, "testing");
	ros::NodeHandle nh_;

	// Subscribers
	ros::Subscriber sub_target = nh_.subscribe("/qualisys/box_target/pose", 1, &targetCallback);
	ros::Subscriber sub_franka_base = nh_.subscribe("/qualisys/mpc_franka/pose", 1, &frankaBaseCallback);
	ros::Subscriber sub_franka = nh_.subscribe("/backstepping/franka_pose", 1, &frankaCallback);

	// Publishers
	ros::Publisher pub_throw = nh_.advertise<std_msgs::Empty>("arduino/blowing_off", 1); 
	ros::Publisher pub_suct = nh_.advertise<std_msgs::Empty>("arduino/suctioning", 1); 
	ros::Publisher pub_reg = nh_.advertise<std_msgs::UInt8>("arduino/duty_cycle", 1); 
	ros::Publisher pub_valve = nh_.advertise<std_msgs::UInt32>("arduino/valve", 1);
	ros::Publisher pub_command = nh_.advertise<panda_controllers::desTrajEE>("/backstepping/command", 1);
	// handtool service
	ros::ServiceClient handtool_client = nh_.serviceClient<handtool_throw::throwing_par_srv>("throwing_par_srv");
	handtool_throw::throwing_par_srv srv;
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
	// wait
	ros::Duration(1.0).sleep();

	std::cout << "Welcome to the hand-tool testing"<< std::endl;
	int choice;
	while(ros::ok()){
		cout<<"choice:   (1: set times (pre-suct,suct),  2: set regulator,  3: set valve time (us),  4: suck&throw,  5: get throw par,  6: throw loop) "<<endl;
		cin>>choice;
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
			// wait
			ros::Duration(pre_suct_time).sleep();
			// suctioning
			pub_suct.publish(empty_msg);
			ros::Duration(suct_time).sleep();
			// throwing
			pub_throw.publish(empty_msg);
			ros::Duration(0.1).sleep();
		}else if (choice == 5){
			// --- get parameters for handtool throwing --- //
			cout << "insert object weight and target x, y, z positions (m_obj x y z):" << endl;
			float m_obj, x, y, z;
			cin >> m_obj;
			cin >> x;
			cin >> y;
			cin >> z;
			srv.request.m_obj = m_obj;
			geometry_msgs::Point target;
			target.x = x;
			target.y = y;
			target.z = z;
			srv.request.target = target;
			if (handtool_client.call(srv)){
				unsigned int valve_opt = srv.response.result_valve_us;
				geometry_msgs::Pose pose = srv.response.result_pose;

				std::cout << "Valve time in us:" << valve_opt << std::endl;
				std::cout << "Position:" << std::endl;
				std::cout << "  x: " << pose.position.x << std::endl;
				std::cout << "  y: " << pose.position.y << std::endl;
				std::cout << "  z: " << pose.position.z << std::endl;
				std::cout << "Orientation:" << std::endl;
				std::cout << "  x: " << pose.orientation.x << std::endl;
				std::cout << "  y: " << pose.orientation.y << std::endl;
				std::cout << "  z: " << pose.orientation.z << std::endl;
				std::cout << "  w: " << pose.orientation.w << std::endl;
			} else {
				ROS_ERROR("Failed to call service");
			}
			ros::Duration(0.1).sleep();
		}else if (choice == 6){
			// - init - //
			has_frankaBase_pose = 0;
			has_franka_pose = false;
			has_target_pose = 0;

			// --- Complete throwing cycle --- //
			// - get mass of object - //
			float m_obj;
			cin >> m_obj;

			// - get target box position - //
			while((!has_target_pose) || (!has_franka_pose)){
				ros::spinOnce();
				ros::Duration(0.01).sleep();
			}

			// - get throwing parameters - //
			srv.request.m_obj = m_obj;
			srv.request.target = target;
			if (handtool_client.call(srv)){
				// unsigned int valve_opt = srv.response.result_valve_us;
				valve_us = srv.response.result_valve_us;
				geometry_msgs::Pose pose = srv.response.result_pose;

				franka_pose.translation() = Eigen::Vector3d(
					pose.position.x,
					pose.position.y,
					pose.position.z);
					
				Eigen::Quaterniond quat(
					pose.orientation.w,
					pose.orientation.x,
					pose.orientation.y,
					pose.orientation.z);
				franka_pose.linear() = quat.toRotationMatrix();

				std::cout << "Valve time in us:" << valve_us << std::endl;
				std::cout << "Position:" << std::endl;
				std::cout << "  x: " << pose.position.x << std::endl;
				std::cout << "  y: " << pose.position.y << std::endl;
				std::cout << "  z: " << pose.position.z << std::endl;
				std::cout << "Orientation:" << std::endl;
				std::cout << "  x: " << pose.orientation.x << std::endl;
				std::cout << "  y: " << pose.orientation.y << std::endl;
				std::cout << "  z: " << pose.orientation.z << std::endl;
				std::cout << "  w: " << pose.orientation.w << std::endl;
			} else {
				ROS_ERROR("Failed to call throw parameters service");
			}
			// - set valve optimal time - //
			valve_msg.data = valve_us;
			pub_valve.publish(valve_msg);

			// --- go to throwing pose --- //
			panda_controllers::desTrajEE cmd;
			cmd.header.stamp = ros::Time::now();

			// - declarations - //
			Eigen::VectorXd pos_start(3), pos_end(3), theta_start(1), theta_end(1);
			Eigen::VectorXd vel_start(3), vel_end(3), dtheta_start(1), dtheta_end(1);
			Eigen::VectorXd pos_t(3), vel_t(3), acc_t(3), theta_t(1), dtheta_t(1), ddtheta_t(1);
			Eigen::VectorXd ZERO_1 = Eigen::VectorXd::Zero(1);
			Eigen::VectorXd ZERO_3 = Eigen::VectorXd::Zero(3);
			Eigen::Quaterniond franka_quat;

			// - common parts - //
			pos_start << franka_pose.translation();
			pos_end << franka_pose_d.translation();
			Eigen::Matrix3d franka_rot = franka_pose.linear().transpose() * franka_pose_d.linear();

			Eigen::AngleAxisd franka_angleAxis(franka_rot);
			double franka_angle = franka_angleAxis.angle();
			Eigen::Vector3d franka_axis = franka_angleAxis.axis();
			theta_start << 0.0;
			theta_end << franka_angle;

			vel_start.setZero();
			vel_end.setZero();
			dtheta_start.setZero();
			dtheta_end.setZero();

			// - trajectory cycle - //
			ros::Rate rate(500);
			double T = 3.0; // trajectory duration
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

				franka_quat = Eigen::Quaterniond(franka_pose.linear()) * Eigen::Quaterniond(Eigen::AngleAxisd(theta_t(0), franka_axis));

				// filling message
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

			// - throw - //
			// wait
			ros::Duration(pre_suct_time).sleep();
			// suctioning
			pub_suct.publish(empty_msg);
			ros::Duration(suct_time).sleep();
			// throwing
			pub_throw.publish(empty_msg);
			
			ros::Duration(0.1).sleep();
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