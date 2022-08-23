#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <algorithm> 
#include <geometry_msgs/Twist.h>

using namespace std;

ros::Publisher points_pub;
ros::Publisher point_min_pub;

const double leafsize = 0.1;
double r;
double min_r;
geometry_msgs::Twist point;

void rs_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  bool isInit = false;
  pcl::PointXYZ p, min_p;
  pcl::PointCloud<pcl::PointXYZ> input_cloud, filtered_cloud;
  input_cloud.header.frame_id = "camera_link";
  filtered_cloud.header.frame_id = "camera_link";
  pcl::fromROSMsg(*input, input_cloud);
  
  
  for(pcl::PointCloud<pcl::PointXYZ>::const_iterator itr = input_cloud.begin(); itr != input_cloud.end(); itr++)
    {
      p.x = (double) itr->z;
      p.y = (double) -itr->x;
      p.z = (double) -itr->y;
      filtered_cloud.push_back(p);
    }


  //平面方程式と平面と検出された点のインデックス
pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

//RANSACによる検出．
pcl::SACSegmentation<pcl::PointXYZ> seg;
seg.setOptimizeCoefficients(true); //外れ値の存在を前提とし最適化を行う
seg.setModelType(pcl::SACMODEL_PLANE); //モードを平面検出に設定
seg.setMethodType(pcl::SAC_RANSAC); //検出方法をRANSACに設定
seg.setDistanceThreshold(0.005); //しきい値を設定
seg.setInputCloud(raw_pointcloud.makeShared()); //入力点群をセット
seg.segment(*inliers, *coefficients); //検出を行う



  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_cloud));
  sensor_msgs::PointCloud2::Ptr tutorial_point_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*filtered_cloud_ptr, *tutorial_point_msg_ptr);
  points_pub.publish(*tutorial_point_msg_ptr);
  
}
  
  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tutorial");
  ros::NodeHandle nh;
  points_pub = nh.advertise<sensor_msgs::PointCloud2> ("/tutorial_points", 1000);
  point_min_pub = nh.advertise<geometry_msgs::Twist>("point_vel", 10);
  ros::Subscriber rs_sub = nh.subscribe("/camera/depth/color/points", 1000, rs_callback);
  ros::spin();
  return 0;
}
