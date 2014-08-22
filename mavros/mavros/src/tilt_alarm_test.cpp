#include <ros/ros.h>
#include <mavros/filteredGvector.h>


class Node {

public:
	Node()
	{
		sub = n.subscribe("/orientation",1000,&Node::run_the_loop,this);
		//pub = n.advertise<mavros::filteredGvector>("orientation",1000);
	}

	void run_the_loop(const mavros::filteredGvector msg);
	//void run_tasks(const sensor_msgs::ImuPtr msg);
	//void read_message(const sensor_msgs::ImuPtr msg);
	//void update_state();
	//void publish_message();

private:
	ros::NodeHandle n;
	//ros::Publisher pub;
	ros::Subscriber sub;

	//message to be read
	mavros::filteredGvector message;


};


void Node::run_the_loop(const mavros::filteredGvector msg)
{
	if(msg.z <= 9.7 ){ROS_INFO("Warning, vehicle has been tilted, z is: %f",msg.z);}
	else{ROS_INFO("all ok");}
}

int main(int argc,char* argv[])
{
	ros::init(argc,argv,"tilt_alarm"); //initialize node, give it a name

	Node myNode;

	ros::spin();

	return 0;

}
