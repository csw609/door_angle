#include "ros/ros.h"
#include <std_msgs/String.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>

std::queue<sensor_msgs::ImageConstPtr> rgb_image_buf;
std::queue<sensor_msgs::ImageConstPtr> depth_image_buf;

void rgb_sub(const sensor_msgs::ImageConstPtr &rgb_image)
{
  rgb_image_buf.push(rgb_image);
}

void depth_sub(const sensor_msgs::ImageConstPtr &depth_image)
{
  depth_image_buf.push(depth_image);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "door");
  ros::NodeHandle nh;
  ros::Subscriber rgb_image_sub = nh.subscribe("camera/infra1/image_rect_raw", 1000, rgb_sub); //config file parameter
  ros::Subscriber depth_image_sub = nh.subscribe("camera/infra2/image_rect_raw", 1000, depth_sub);

  ros::Publisher img_match_pub = nh.advertise<sensor_msgs::Image>("match_image", 1000);

  sensor_msgs::Image match_image;
  cv_bridge::CvImage cv_bridge;
  std_msgs::Header header;

  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    if (!rgb_image_buf.empty() && !depth_image_buf.empty())
    {
      double time;
      double time_l = rgb_image_buf.front()->header.stamp.toSec();
      double time_r = depth_image_buf.front()->header.stamp.toSec();
      cv::Mat rgb_image;
      cv::Mat depth_image;

      //rgb, depth image time sync 0.003
      if (time_l < time_r - 0.003)
      {
        rgb_image_buf.pop();
        ROS_INFO("pop rgb_image\n");
      }
      else if (time_l > time_r + 0.003)
      {
        depth_image_buf.pop();
        ROS_INFO("pop depth_image\n");
      }
      else
      {
        time = rgb_image_buf.front()->header.stamp.toSec();

        //sensor_msgs::image to cv::Mat
        cv_bridge::CvImageConstPtr ptr_l;
        ptr_l = cv_bridge::toCvCopy(rgb_image_buf.front(), sensor_msgs::image_encodings::MONO8);
        cv_bridge::CvImageConstPtr ptr_r;
        ptr_r = cv_bridge::toCvCopy(depth_image_buf.front(), sensor_msgs::image_encodings::MONO8);

        rgb_image = ptr_l->image.clone();
        rgb_image_buf.pop();
        depth_image = ptr_r->image.clone();
        depth_image_buf.pop();
      }

      if (!rgb_image.empty() && !depth_image.empty())
      {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;




      }

      ros::spinOnce();
    }

  }


  return 0;
}
