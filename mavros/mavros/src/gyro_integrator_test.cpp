/*
 * gyro_integrator_test.cpp
 *
 *  Created on: Aug 15, 2014
 *      Author: roman
 */
/*
 * This is an experiment on how to interface the PIXHAWK board to a device running a ROS environment.
 * Data from the PIXHAWK are received via UART and are handled by a mavros node.
 * This file runs a node which fetches sensor data from a mavros topic and performs some basic filtering on it.
 * The results are then published on the topic '/orientation'.
 * The main function of this file initializes the Node once. Once data is available in the mavros topic this Node is used to run
 * all desired tasks.
 *
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <mavros/filteredGvector.h>

#define g 9.81	//gravity

//filter params
float weighting_gyro = 0.9;
float weighting_accel = 1 - weighting_gyro;


class Node {

public:
	Node()
	{
		sub = n.subscribe("mavros/imu/data_raw",1000,&Node::run_the_loop,this);
		pub = n.advertise<mavros::filteredGvector>("orientation",1000);
	}

	void run_the_loop(const sensor_msgs::ImuPtr msg);
	void run_tasks(const sensor_msgs::ImuPtr msg);
	void read_message(const sensor_msgs::ImuPtr msg);
	void update_state();
	void publish_message();

private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;

	//message to be published
	mavros::filteredGvector message;


};


struct gyro {
	float x;
	float y;
	float z;
}mygyro;

struct accel {
	float x;
	float y;
	float z;
}myaccel;


struct pos_state {
	float x;
	float y;
	float z;
}pos;

void
Node::update_state()
{
	//Prediction
	pos.x = pos.x + (mygyro.y*pos.z - mygyro.z*pos.y)*0.02;
	pos.y = pos.y + (mygyro.z*pos.x - mygyro.x*pos.z)*0.02;
	pos.z = pos.z + (mygyro.x*pos.y - mygyro.y*pos.x)*0.02;

	//Measurement update
	pos.x = weighting_gyro * pos.x + weighting_accel * myaccel.x;
	pos.y = weighting_gyro * pos.y + weighting_accel * myaccel.y;
	pos.z = weighting_gyro * pos.z + weighting_accel * myaccel.z;

	//Normalization:Gravity vector should always have same length
	float mag = sqrt(pos.x*pos.x + pos.y*pos.y + pos.z*pos.z);
	pos.x = pos.x/mag*g;
	pos.y = pos.y/mag*g;
	pos.z = pos.z/mag*g;

}

void Node::run_the_loop(const sensor_msgs::ImuPtr msg)
{
	run_tasks(msg);
}

void Node::run_tasks(const sensor_msgs::ImuPtr msg)
{
	read_message(msg);
	update_state();
	publish_message();
}

void Node::read_message(const sensor_msgs::ImuPtr msg)
{
	mygyro.x = msg->angular_velocity.x;
	mygyro.y = msg->angular_velocity.y;
	mygyro.z = msg->angular_velocity.z;

	myaccel.x = msg->linear_acceleration.x;
	myaccel.y = msg->linear_acceleration.y;
	myaccel.z = msg->linear_acceleration.z;
}

void Node::publish_message()
{
	message.x = pos.x;
	message.y = pos.y;
	message.z = pos.z;

	pub.publish(message);
}

int main(int argc, char** argv)
{

	pos.x = 0;
	pos.y = 0;
	pos.z = g;

	ros::init(argc,argv,"gyro_integrator");

	Node myNode;

	ros::spin();

	return 0;


}
