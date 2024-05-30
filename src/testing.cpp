#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>

// #include <tf/transform_broadcaster.h>
// #include <geometry_msgs/Pose.h>
// #include <eigen_conversions/eigen_msg.h>

// #include<tf_conversions/tf_eigen.h>

using namespace std;

// int counter = 0;
std_msgs::Empty empty_msg;
std_msgs::UInt32 valve_msg;
std_msgs::UInt8 regulator_msg;

float pre_suct_time = 2;
float suct_time = 0.5;
unsigned int valve_us = 50000; 	// valve time opering in microseconds
unsigned int duty_regulator = 255; // 255: bypass regulator


int main(int argc, char **argv)
{       
	ros::init(argc, argv, "testing");
	ros::NodeHandle nh_;
	 
	// Define publishers
	ros::Publisher pub_throw = nh_.advertise<std_msgs::Empty>("arduino/blowing_off", 1); 
	ros::Publisher pub_suct = nh_.advertise<std_msgs::Empty>("arduino/suctioning", 1); 
	ros::Publisher pub_reg = nh_.advertise<std_msgs::UInt8>("arduino/duty_cycle", 1); 
	ros::Publisher pub_valve = nh_.advertise<std_msgs::UInt32>("arduino/valve", 1);
	ros::Duration(1.0).sleep();

	std::cout << "Welcome to the hand-tool testing"<< std::endl;
	int choice;
	while(ros::ok()){
		cout<<"choice:   (1: set times (pre-suct,suct),  2: set regulator,  3: set valve time (us),  4: throw loop) "<<endl;
		cin>>choice;
		// while (!ready) ros::spinOnce();
		if (choice == 1){
			// --- set times --- //
			cin >> pre_suct_time;
			cin >> suct_time;
			ros::Duration(0.1).sleep();
			cout << "ok!" << endl;
		}else if (choice == 2){
			// --- set duty-cycle of regulator --- //
			cin >> duty_regulator;
			regulator_msg.data = duty_regulator;
			pub_reg.publish(regulator_msg);
			ros::Duration(0.1).sleep();
			cout << "ok!" << endl;
		}else if (choice == 3){
			// --- set valve time --- //
			cin >> valve_us;
			valve_msg.data = valve_us;
			pub_valve.publish(valve_msg);
			ros::Duration(0.1).sleep();
			cout << "ok!" << endl;
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
		}
	}
}