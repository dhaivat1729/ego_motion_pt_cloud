/*

This code first subscribes to a point cloud and a pose. This part runs for once only. For the rest of the code, this information
will be used to transform point cloud to the new poses which will keep coming up! Along with that, we will get new poses, left
image and right image in a synchronized manner. We will transform this a point cloud we received first in the reference frame
of the new pose available.

*/



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
#include <message_filters/connection.h>
#include <string>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/io/pcd_io.h>
#include <unistd.h>
#include <pcl/io/ply_io.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;
using namespace geometry_msgs;
using namespace pcl;
using namespace pcl_ros;
using namespace tf2;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);    // The input PointCloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_tf (new pcl::PointCloud<pcl::PointXYZRGB>); // The input pointcloud transformed in cam0's new frame
geometry_msgs::Pose root_pose;
geometry_msgs::Pose instant_pose;
tf::Pose rp;  // root pose
tf::Pose ip;  // instant pose
tf::Pose net_transform;
tf::Quaternion q; // rotation matrix
tf::Vector3 v;  // translation matrix
tf::Quaternion new_pose_q; // rotation matrix
tf::Vector3 new_pose_v;  // translation matrix

bool flag = true;
int i = 1;
string base_path = "/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/computation_optimization_workspace/src/subscriber_pkg/Point_clouds2/";
string save_right_image = "/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/computation_optimization_workspace/src/subscriber_pkg/right_raw/";
string save_left_image = "/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/computation_optimization_workspace/src/subscriber_pkg/left_raw/";

void callback(const sensor_msgs::PointCloud2::ConstPtr& pt_cloud_msg, const geometry_msgs::PoseStamped::ConstPtr& pose_msg){


  if(flag){
  // Converting pointcloud from ROS message type to pcl type!
  pcl::fromROSMsg (*pt_cloud_msg, *cloud_in);

  // Converting Ros pose msg to normal pose
  root_pose = pose_msg->pose;
  tf::poseMsgToTF(root_pose, rp);
  //Transform(Quaternion(pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z, pose_msg->pose.orientation.w), Vector3(pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z));
  net_transform = rp.inverseTimes(rp);
  q = net_transform.getRotation();
  v = net_transform.getOrigin ();
  cout << "(" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << ")" << endl;
  cout << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")" << endl;
  flag = false;
  pcl::io::savePLYFileASCII (base_path + "Input_pcd.ply", *cloud_in);
  }
  return;
}

void sync_callback(const ImageConstPtr& image_left, const ImageConstPtr& image_right, const geometry_msgs::PoseStamped::ConstPtr& msg){

   cv_bridge::CvImagePtr cv_ptr_left;
   cv_bridge::CvImagePtr cv_ptr_right;
   cv_ptr_left = cv_bridge::toCvCopy(image_left, sensor_msgs::image_encodings::BGR8);
   cv_ptr_right = cv_bridge::toCvCopy(image_right, sensor_msgs::image_encodings::BGR8);
   cout << "Inside sync_callback!" << endl;
   cout << "( " << msg->pose.position.x << ", " << msg->pose.position.y << ", " << msg->pose.position.z << ")" << endl;
   tf::poseMsgToTF(msg->pose, ip);
	 net_transform = rp.inverseTimes(ip); // Instant pose(ip) - root pose(rp)
   q = net_transform.getRotation();
   v = net_transform.getOrigin ();
   cout << "(" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << ")" << endl;
   cout << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")" << endl;
   pcl_ros::transformPointCloud(*cloud_in, *cloud_in_tf, net_transform);
   stringstream ss;
   ss << i;
   string str = ss.str();
   if(cloud_in_tf->size() > 0)  {
     imwrite(save_left_image + "left_" + str + ".ppm", cv_ptr_left->image);
     imwrite(save_right_image + "right_" + str + ".ppm", cv_ptr_right->image);
     pcl::io::savePLYFileASCII (base_path + "Output_" + str + ".ply", *cloud_in_tf);
     i = i + 1;
   }


}

int main(int argc, char** argv){

    ros::init(argc, argv, "SyncedPoseImages");

    ros::NodeHandle node_object;
    message_filters::Subscriber<sensor_msgs::PointCloud2> Pointcloud_sub(node_object, "/pcl_xyzrgb", 1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> Pose_sub(node_object, "/pose", 1);
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), Pointcloud_sub, Pose_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Subscribing to right raw image
    message_filters::Subscriber<Image> right_raw_sub(node_object, "/right_rgb/image", 1);

    // Subscriber for left raw image
    message_filters::Subscriber<Image> left_raw_sub(node_object, "/left_rgb/image", 1);

    // Subscribing for new pose
    message_filters::Subscriber<geometry_msgs::PoseStamped> Pose_new_sub(node_object, "/pose", 1);
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, geometry_msgs::PoseStamped> MySyncPolicy2;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy2> sync2(MySyncPolicy2(10), left_raw_sub, right_raw_sub, Pose_new_sub);
    sync2.registerCallback(boost::bind(&sync_callback, _1, _2, _3));


    //ros::Subscriber pose_subscriber = node_object.subscribe("/pose", 10, pose_callback);
    ros::spin();
    return 0;
}
