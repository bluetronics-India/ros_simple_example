/* arduino.cpp

   Quick attempt at turning the data from the arduino into a laser message

*/

#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>
#include <stdlib.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Quaternion.h"

uint32_t id = 0;
ros::Publisher laserpub;
ros::Publisher odom_pub;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void arduinoCallback(const std_msgs::String::ConstPtr& msg)
{

	static tf::TransformBroadcaster br;

  	tf::Transform transform;
	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	transform.setRotation( tf::Quaternion(0, 0, 0) );
	//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "laser"));
	//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));


//next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;

    geometry_msgs::Quaternion rosQuat;
    rosQuat.x = 0.0;
    rosQuat.y = 0.0;
    rosQuat.z = 0.0;
    rosQuat.w = 0.0;
    odom.pose.pose.orientation = rosQuat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;
    
    //publish the message
   // odom_pub.publish(odom);
/* 
		min_height_(0.10),
                 max_height_(0.15),
                 angle_min_(-M_PI/2),
                 angle_max_(M_PI/2),
                 angle_increment_(M_PI/180.0/2.0),
                 scan_time_(1.0/30.0),
                 range_min_(0.45),
                 range_max_(10.0),
                 output_frame_id_("/kinect_depth_frame")
*/
	//Got data in.
	//printf("[%s]\n", msg->data.c_str());

	sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());

	output->header.stamp = ros::Time::now();
	output->header.seq = id++;
    output->header.frame_id = "laser"; //associated with laser
    output->angle_min = -1.04719755; //-60 degrees
    output->angle_max = 1.04719755; //+60 degrees
    output->angle_increment = 0.0872664626; //5 Degrees
    output->time_increment = 0.0; //tbc
    output->scan_time = (2.0); //tbc
    output->range_min = 0.01; //1cm
    output->range_max = 3.0; //3m


	uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
    output->ranges.assign(ranges_size, output->range_max + 1.0);
	//float32[] ranges;

	//Prove we can split the data
	std::istringstream oss(std::string(msg->data.c_str()));
    std::string word;
	int index = 0;
    while(getline(oss, word, ',')) {
		output->ranges[index] = atof(word.c_str()) / 100.0;
		//printf("%s\n", word.c_str());
		index++;
	}

	laserpub.publish(output);
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "arduinotolaser");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber arduinoSub = n.subscribe("arduino_data", 100, arduinoCallback);
  laserpub = n.advertise<sensor_msgs::LaserScan>("scan", 100);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
