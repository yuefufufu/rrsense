#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm> 
#include <geometry_msgs/Twist.h>

using namespace std;

ros::Publisher points_pub;
ros::Publisher point_min_pub;

const double leafsize = 0.1;
double r[51] ={100.0};
double min_r[51]={0};
geometry_msgs::Twist point;

void rs_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  bool isInit = false;
  pcl::PointXYZ p, min_p[51];
  pcl::PointCloud<pcl::PointXYZ> input_cloud, filtered_cloud;
  input_cloud.header.frame_id = "camera_link";
  filtered_cloud.header.frame_id = "camera_link";
  pcl::fromROSMsg(*input, input_cloud);
  
  //cout << "Raw data size: " << input_cloud.size() << endl;
  
  for(pcl::PointCloud<pcl::PointXYZ>::const_iterator itr = input_cloud.begin(); itr != input_cloud.end(); itr++)
    {
      p.x = (double) itr-> x;
      p.y = (double) -itr-> y*-1;
      p.z = (double) -itr-> z*-1;
      if(p.x > 0)
      {
        filtered_cloud.push_back(p);
      }

      r[0] = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
      for(int i=0;i<30;i++)
      {
        if(r[i]<r[i+1]);
        {
          std::swap(r[i+1], r[i]);
          min_p[i+1]=p;
        }
      }

    }

  cout << min_p[1] << endl;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_cloud));

  sensor_msgs::PointCloud2::Ptr tutorial2_point_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*filtered_cloud_ptr, *tutorial2_point_msg_ptr);
  points_pub.publish(*tutorial2_point_msg_ptr);
  
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tutorial2");
  ros::NodeHandle nh;
  points_pub = nh.advertise<sensor_msgs::PointCloud2> ("/tutorial2_points", 1000);
  ros::Subscriber rs_sub = nh.subscribe("/tutorial_points", 1000, rs_callback);
  ros::spin();
  return 0;
}