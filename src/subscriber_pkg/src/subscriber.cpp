#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf/transform_listener.h>
#include <sstream> // for ostringstream
#include <string>

using namespace sensor_msgs;
using namespace message_filters;
using namespace std;
using namespace geometry_msgs;

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){

	ROS_INFO("Received! [%f]", msg->pose.orientation.w*msg->pose.orientation.w + msg->pose.orientation.x*msg->pose.orientation.x + msg->pose.orientation.y*msg->pose.orientation.y + msg->pose.orientation.z*msg->pose.orientation.z);

}


void synccallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg, const sensor_msgs::Image::ConstPtr& image_msg){
	
	ROS_INFO("Received! [%f]", pose_msg->pose.orientation.w*pose_msg->pose.orientation.w + pose_msg->pose.orientation.x*pose_msg->pose.orientation.x + pose_msg->pose.orientation.y*pose_msg->pose.orientation.y + pose_msg->pose.orientation.z*pose_msg->pose.orientation.z);
	

}

int main(int argc, char** argv){

	ros::init(argc, argv, "Pose_subcriber");
	ros::NodeHandle node_obj;
	message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(node_obj, "/pose", 1);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(node_obj, "/depth_map/image", 1);

	//typedef sync_policies::ApproximateTime<PoseStamped, Image> MySyncPolicy;
	typedef sync_policies::ExactTime<PoseStamped, Image> MySyncPolicy;	
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pose_sub, depth_sub);
	sync.registerCallback(boost::bind(&synccallback, _1, _2));

	//TimeSynchronizer<geometry_msgs::PoseStamped, sensor_msgs::Image> sync(pose_sub, depth_sub, 10);
	//sync.registerCallback(boost::bind(&pose_callback, _1, _2));
	//ros::Subscriber pose_subscriber = node_obj.subscribe("/pose", 10, pose_callback);
	ros::spin();
	return 0;
}

