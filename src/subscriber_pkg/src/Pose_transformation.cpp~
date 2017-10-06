/*
 *  This code subscribes to point cloud and pose at the same time first. That callback is run once only.(Function name: callback)
 *  The point cloud we received is root point cloud and the pose we received is known as root pose.
 *  Later, we only subscribe to the new pose.(callback name: pose_callback)
 *  Everytime new pose arrives, we find the transformation from root pose to new pose. (new_pose - root_pose)
 *  Once we get the transformation, we transform the root point cloud in new pose's frame of reference!
 */


#include <ros/ros.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/core/core.hpp>
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
#include <pcl/common/common.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace std;
using namespace geometry_msgs;
using namespace pcl;
using namespace pcl_ros;
using namespace tf2;
using namespace cv;

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
tf::Matrix3x3 m;

bool flag = true;
int i = 1;
string base_path = "/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/computation_optimization_workspace/src/subscriber_pkg/Point_clouds/";
string depth_grid_path = "/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/computation_optimization_workspace/src/subscriber_pkg/depth_grid/";

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

  // The pointcloud coming has Z in outward direction, X in right direction and Y in downward direction as per the right hand rule!
  // Now the pose is coming in the frame where X is in outward direction, Y in left and Z in upward as per the right hand rule!
  // We will transform the pointcloud in the co-ordinate system of pose.
  // From pointcloud's coordinate system to pose's co-ordinate system, the converion is as below.
  // X = Z, Z = -Y, Y = -X
  // In order to achieve this, we have to rotate the pointcloud around (1,0,0) by -90 degrees in clockwise(90 degrees counter clockwise). and then we have to transform the new pointcloud
  // around (0,0,1) by -90 degrees in clockwise direction!
  // Here when we want to rotate around (1,0,0) by 90 degrees in clockwise, we essentially give -90, because the traditional convention assumes counter clockwise rotation.


  tf::Quaternion q1(-0.7071, 0, 0, 0.7071); // theta = 90, theta/2 = 45 degrees (x,y,z,w), Around (-1,0,0) which is same as rotation around (1,0,0) by -90 degrees in clockwise
  tf::Vector3 t1(0,0,0); // Translation is zero!
  tf::Quaternion q2(0, 0, -0.7071, 0.7071); // Same as above!
  tf::Vector3 t2(0,0,0);
  tf::Transform tf1(q1, t1); // Around (1,0,0) by 90 degrees
  tf::Transform tf2(q2, t2); // Around (0,0,1) by 90 degrees
  pcl::io::savePLYFileASCII (base_path + "Input_pcd_original.ply", *cloud_in);
  // This function does RX + T
  pcl_ros::transformPointCloud(*cloud_in, *cloud_in, tf1);
  pcl_ros::transformPointCloud(*cloud_in, *cloud_in, tf2);
  //cout << "(" << q1.w() << ", " << q1.x() << ", " << q1.y() << ", " << q1.z() << ")" << endl;
  //cout << "(" << t1.x() << ", " << t1.y() << ", " << t1.z() << ")" << endl;
  flag = false;
  pcl::io::savePLYFileASCII (base_path + "Input_pcd.ply", *cloud_in);
  }
  return;
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){

   //cout << "Inside pose_callback!" << endl;
   //cout << "( " << msg->pose.position.x << ", " << msg->pose.position.y << ", " << msg->pose.position.z << ")" << endl;
   tf::poseMsgToTF(msg->pose, ip);
   net_transform = ip.inverseTimes(rp); // Instant pose(ip) - root pose(rp)
   q = net_transform.getRotation();
   v = net_transform.getOrigin ();

   pcl_ros::transformPointCloud(*cloud_in, *cloud_in_tf, net_transform);

   // basically inverse tranform of what we did in callback!
   tf::Quaternion q3(0, 0, 0.7071, 0.7071); // theta = -90, theta/2 = 45 degrees (x,y,z,w), Around (0,0,-1) which is same as rotation around (0,0,1) by -90 degrees in clockwise
   tf::Vector3 t3(0,0,0); // Translation is zero!
   tf::Quaternion q4(0.7071, 0, 0, 0.7071); // Same as above!
   tf::Vector3 t4(0,0,0);
   tf::Transform tf3(q3, t3); // Around (0,0,-1) by -90 degrees clockwise
   tf::Transform tf4(q4, t4); // Around (1,0,0) by -90 degrees clockwise
   // This function does RX + T
   pcl_ros::transformPointCloud(*cloud_in_tf, *cloud_in_tf, tf3);
   pcl_ros::transformPointCloud(*cloud_in_tf, *cloud_in_tf, tf4);

   stringstream ss;
   ss << i;
   string str = ss.str();
   if(cloud_in_tf->size() > 0)  {
     pcl::io::savePLYFileASCII (base_path + "Output_" + str + ".ply", *cloud_in_tf);
     cout << str << " ";
     cout << "Rotation q(w,x,y,z) = " << "(" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << ")." << endl;
     cout << "  Translation t(x,y,z) = " <<"(" << v.x() << ", " << v.y() << ", " << v.z() << ")" << endl;
	    // Defining ranges of scene we want to cover
    double max_x = 4.0;
    double min_x = -4.0;
    double max_y = 4.0;
    double min_y = -4.0;
    double max_z = 5.1;
    double min_z = 0.1; // Can't be 0!
	
    //cout<<max_x<<" "<<max_y<<" "<<max_z<<" "<<min_x<<" "<<min_y<<" "<<min_z<<endl;
    double  interval_x = (max_x - min_x)/100.0;  // X values along horizontal axis
    double  interval_y = (max_y - min_y)/100.0;
    double  interval_z = (max_z - min_z)/255.0; // Z values along vertical axis
     //min_x=abs(min_x);
     //min_y=abs(min_y);
     //min_z=abs(min_z);
    Mat depth_grid(100, 100, CV_8UC1, Scalar(255));
    int row, col, depth_value;
   int non_zero_depth = 0;
   cout << "Point cloud size is: " << cloud_in_tf->points.size() << endl;	
   // From the transformed point cloud, we are trying to estimate the depth grid 
    for(size_t  f=0; f < cloud_in_tf->points.size();++f)
      {

        if((cloud_in_tf->points[f].x < max_x && cloud_in_tf->points[f].x > min_x) && (cloud_in_tf->points[f].y < max_y && cloud_in_tf->points[f].y > min_y) && (cloud_in_tf->points[f].z < max_z && cloud_in_tf->points[f].z > min_z)){
        double X=cloud_in_tf->points[f].x;
        double Y=cloud_in_tf->points[f].y;
        double Z=cloud_in_tf->points[f].z;
	row = (int)((X - min_x)/interval_x);	
	col = (int)((Y - min_y)/interval_y);
	depth_value = 255 - (int)((Z - min_z)/interval_z);	 
        depth_grid.at<uchar>(row,col) = depth_value;
	if(depth_value != 255 ) non_zero_depth += 1;
      }

    }
    imwrite(depth_grid_path + "Output_" + str + ".png", depth_grid);	
    i += 1;
    cout << "Nunmber of non zero depth pixels are: " << non_zero_depth << endl;
   }

}

int main(int argc, char** argv){

    ros::init(argc, argv, "PoseTransformation");
    ros::NodeHandle node_object;
    message_filters::Subscriber<sensor_msgs::PointCloud2> Pointcloud_sub(node_object, "/pcl_xyzrgb", 1);
    message_filters::Subscriber<geometry_msgs::PoseStamped> Pose_sub(node_object, "/pose", 1);
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), Pointcloud_sub, Pose_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::Subscriber pose_subscriber = node_object.subscribe("/pose", 10, pose_callback);
    ros::spin();
    return 0;
}
