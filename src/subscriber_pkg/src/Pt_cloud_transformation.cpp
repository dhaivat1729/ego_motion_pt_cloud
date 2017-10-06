#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf/transform_listener.h>
#include <sstream> // for ostringstream
#include <string>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/io/pcd_io.h>
#include <unistd.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace std;
using namespace geometry_msgs;
using namespace pcl;
using namespace pcl_ros;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);    // The input PointCloud
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_tf (new pcl::PointCloud<pcl::PointXYZ>); // The input pointcloud transformed in cam0's new frame

// Use pcl_conversion to convert from sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ> cloud;!! To be done.

// A callback function of a point cloud subsciber
void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

     //ROS_INFO("inside callback");

     // Converting pointcloud from ROS message type to pcl type!
     pcl::fromROSMsg (*cloud_msg, *cloud_in);

     // A listener which would wait and listen for transformations as and when it comes!!!1
     tf::TransformListener listener;

     // A variable where upcoming transform is going to be stored
     tf::StampedTransform transform;


     tf2_ros::Buffer tfBuffer;
     tf2_ros::TransformListener tfListener(tfBuffer);
     geometry_msgs::TransformStamped transformStamped;

    // A timestamp of the pointcloud will be stored in this particular variable called pt_cloud_ts
     ros::Time pt_cloud_ts = cloud_msg->header.stamp;  // Point  cloud timestamp
     cout << "Before sleeping!" << endl;
     usleep(2000000);
     cout << "After sleeping!" << endl;
     /*
     // What is the pose of /map at pt_cloud_ts with respect to position of /cam0 at time pt_cloud_ts!
     try{
     listener.waitForTransform("/cam0", "/map", pt_cloud_ts, ros::Duration(0.001));
     listener.lookupTransform("/cam0", "/map", pt_cloud_ts, transform);
      }
      */
      /*************************************************************************
       *    This is just for understanding of the coder!
       *    What we really want to ask is, "What was the pose of /turtle1 5 seconds ago, relative to the current position of the /turtle2?"
       *    try{
       *      ros::Time now = ros::Time::now();
       *      ros::Time past = now - ros::Duration(5.0);
       *       listener.waitForTransform("/turtle2", now,
       *                        "/turtle1", past,
       *                        "/world", ros::Duration(1.0));
       *      listener.lookupTransform("/turtle2", now,
       *                        "/turtle1", past,
       *                        "/world", transform);
      *************************************************************************/

     // Similarly, our question is, what is the current position of the /camera0, relative
     // to the position of the camera0 at the time when we received the pointcloud?
     try{

      /*
      This block waits for the transformation.
      Here we are looking for a transformation as mentioned in the comment immediately before try block!
      */
     // Here, it has been assumed that /map frame is constant.
     // /map frame is generally initialized when SLAM system initializes
     // Here, we will wait for 2 seconds to get the transformation! Hence the last argument is ros::Duration(2.0)!
     // /map is the only constant frame in the process!
     listener.waitForTransform("/cam0", pt_cloud_ts, "/cam0", ros::Time(0), "/map", ros::Duration(2.0));
     listener.lookupTransform("/cam0", pt_cloud_ts, "/cam0", ros::Time(0), "/map", transform);
     cout  << "We wanted transformation data for timestamp of " << pt_cloud_ts << " and we received for " <<  pt_cloud_ts << "!" << endl;
   }
     catch (tf::TransformException ex) {

       // It reaches inside this block if it fails to get the transformation within the timeout limit we have provided!
       //ROS_ERROR("%s",ex.what());

       // The error message contains a string.
       /************************************************************************
        * Here is what is done in the rest of the part of the catch block!
        * Sometimes it happens that transformation for a particular timestamp
        * might not be available but transformation for a nearer transformation
        * could be available!
        * We essentially extract the timestamp from the exception message for
        * which the transformation is available!
        ***********************************************************************/

       // Getting the error message!
       string error = ex.what();
       cout << error << endl;
       // Incase exception is about target frame not available, we can't do anything. Hence we ignore it.
       // The code goes inside if statement if and only if the exception is about transformation being available for the nearest timestamp!
       // error.length() > 140 signifies that if the error length is greater than 140 characters, we know that the error is about transformation being
       // available for the nearest frame!
       if(error.length() > 140) {

         // Getting the timestamp for which transformation is available.
         // The ideal error message would look something like the following line!
         // Lookup would require extrapolation into the past.  Requested time 1501768003.912780912 but the earliest data is at time 1501768004.379579912, when looking up transform from frame [map] to frame [cam0]
         std::string secs = error.substr(120,10);   // the seconds part of timestamp is between characters 120 to 130
         std::string Nsecs = error.substr(131,9);   // the nano seconds part of the timestamp is between characters 131 to 140.

         // Getting seconds in unit_32 format
         std::istringstream reader1(secs);
         unsigned int seconds;
         reader1 >> seconds;

         // Getting nano seconds in unit_32 format
         std::istringstream reader2(Nsecs);
         unsigned int Nseconds;
         reader2 >> Nseconds;

         // Once we get the timestamp, we store it in real_time_stamp variable. Later we use it to get the transformation which is available for that timestamp
         ros::Time real_time_stamp(seconds,  Nseconds);
         listener.waitForTransform("/cam0", real_time_stamp, "/cam0", ros::Time(0), "/map", ros::Duration(0.5));
         listener.lookupTransform("/cam0", real_time_stamp, "/cam0", ros::Time(0), "/map", transform);
         cout << "The difference between desired timestamp and received timestamp is: " << pt_cloud_ts - real_time_stamp << "!" << endl;
         cout  << "We wanted transformation data for timestamp of " << pt_cloud_ts << " but we received for " <<  real_time_stamp << "!" << endl;
       }
      }

      // transforming pointcloud to new frame of reference once transformation is received
      tf::Quaternion q = transform.getRotation ();
      Eigen::Quaternionf rotation(q.w (), q.x (), q.y (), q.z ());       // internally stored as (x,y,z,w)
      tf::Vector3 v = transform.getOrigin ();
      Eigen::Vector3f origin(v.x (), v.y (), v.z ());
      //tf2::doTransform(*cloud_in, *cloud_in_tf, transform);
      pcl_ros::transformPointCloud(*cloud_in, *cloud_in_tf, transform);
      //pcl::io::savePCDFileASCII ("Input_pcd.pcd", *cloud_in);
      //pcl::io::savePCDFileASCII ("Input_pcd_tf.pcd", *cloud_in_tf);
      //exit(0);
}

int main(int argc, char** argv){

	// Initializing ROS_environment
	ros::init(argc, argv, "Pt_cloud_transformation");

  // Ros node handler
	ros::NodeHandle node_obj;
	//ros::Rate loop_rate(10);

  // A subscriber to subscribe to point cloud
	ros::Subscriber sub;

	cout << "Before callback!" << endl;

  // Getting a point cloud through subscriber!
  sub = node_obj.subscribe ("/pcl_xyzrgb", 10, cloud_callback);
	ros::spin();
	return 0;
}
