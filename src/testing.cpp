#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include "handtool_throw/throwing_par_srv.h"

// #include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
// #include <eigen_conversions/eigen_msg.h>

// #include<tf_conversions/tf_eigen.h>

using namespace std;

// int counter = 0;
std_msgs::Empty empty_msg;
std_msgs::UInt32 valve_msg;
std_msgs::UInt8 regulator_msg;

double pre_suct_time = 2.0;
double suct_time = 0.5;
int valve_us = 50000; 	// valve time opering in microseconds
int duty_regulator = 255; // 255: bypass regulator


int main(int argc, char **argv)
{       
	ros::init(argc, argv, "testing");
	ros::NodeHandle nh_;
	 
	// Define publishers
	ros::Publisher pub_throw = nh_.advertise<std_msgs::Empty>("arduino/blowing_off", 1); 
	ros::Publisher pub_suct = nh_.advertise<std_msgs::Empty>("arduino/suctioning", 1); 
	ros::Publisher pub_reg = nh_.advertise<std_msgs::UInt8>("arduino/duty_cycle", 1); 
	ros::Publisher pub_valve = nh_.advertise<std_msgs::UInt32>("arduino/valve", 1);
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
		cout<<"choice:   (1: set times (pre-suct,suct),  2: set regulator,  3: set valve time (us),  4: throw loop,  5: get_throw_par) "<<endl;
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
			// --- set duty-cycle of regulator --- //
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
		}
	}
}