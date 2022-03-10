#include "ros/ros.h"
#include <std_msgs/String.h>

#include <iostream>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "door_angle/bounding.h"
#include "door_angle/BoundingBox.h"
#include "door_angle/BoundingBoxes.h"

std::queue<sensor_msgs::ImageConstPtr>     rgb_image_buf;
std::queue<sensor_msgs::ImageConstPtr>     depth_image_buf;
//std::queue<door_angle::bounding::ConstPtr> bounding_buf;
std::queue<door_angle::BoundingBoxesPtr> bounding_buf;

float fx = 610.8421650074351;  // need rgb calib
float cx = 336.7626691266633;  // need rgb calib
float cy = 252.1813169593813; // need rgb calib

void rgb_sub(const sensor_msgs::ImageConstPtr &rgb_image)
{
  rgb_image_buf.push(rgb_image);
}

void depth_sub(const sensor_msgs::ImageConstPtr &depth_image)
{
  depth_image_buf.push(depth_image);
}

void bounding_sub(const door_angle::BoundingBoxesPtr &msg)
{
  bounding_buf.push(msg);
}

int main(int argc, char **argv)
{
  //init
  ros::init(argc, argv, "door");
  ros::NodeHandle nh;

  // subscriber
  ros::Subscriber rgb_image_sub   = nh.subscribe("/camera/color/image_raw", 1000, rgb_sub);
  ros::Subscriber depth_image_sub = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1000, depth_sub);
  ros::Subscriber bouding_box_sub = nh.subscribe("/bounding_box",1000, bounding_sub);

  // publisher
  ros::Publisher img_match_pub  = nh.advertise<sensor_msgs::Image>("match_image", 1000);
  ros::Publisher pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("door", 1000);

  //variable
  sensor_msgs::Image match_image;
  cv_bridge::CvImage cv_bridge;
  std_msgs::Header header;
  bool first = true;
  sensor_msgs::PointCloud2 cloudmsg;

  ros::Rate loop_rate(15);
  while (ros::ok())
  {
    if (!rgb_image_buf.empty() && !depth_image_buf.empty() && !bounding_buf.empty())
    {
      //ROS_INFO("buf no empty");

      double time;
      double time_r = rgb_image_buf.front()->header.stamp.toSec();
      double time_d = depth_image_buf.front()->header.stamp.toSec();
      double time_b = bounding_buf.front()->header.stamp.toSec();

      cv::Mat rgb_image;
      cv::Mat depth_image;
      door_angle::BoundingBox box;

      //rgb, depth, bounding box time sync 0.003
      if (time_r < time_d - 0.003)
      {
        rgb_image_buf.pop();
        ROS_INFO("pop rgb_image\n");
      }
      else if (time_r > time_d + 0.003)
      {
        depth_image_buf.pop();
        ROS_INFO("pop depth_image\n");
      }
      else if(time_r < time_b - 0.003){
        rgb_image_buf.pop();
        ROS_INFO("pop rgb_image\n");
      }
      else if(time_r > time_b + 0.003){
        bounding_buf.pop();
        ROS_INFO("pop bound header\n");
      }
      else if(!bounding_buf.front()->bounding_boxes.empty())  //bounding box not empty && satisfy time sync
      {
        time = rgb_image_buf.front()->header.stamp.toSec();

        //convert sensor_msgs::image to cv::Mat
        cv_bridge::CvImageConstPtr ptr_l;
        ptr_l = cv_bridge::toCvCopy(rgb_image_buf.front(), sensor_msgs::image_encodings::BGR8);
        cv_bridge::CvImageConstPtr ptr_r;
        ptr_r = cv_bridge::toCvCopy(depth_image_buf.front(), sensor_msgs::image_encodings::TYPE_16UC1);

        rgb_image = ptr_l->image.clone();
        rgb_image_buf.pop();
        depth_image = ptr_r->image.clone();
        depth_image_buf.pop();

      }

      if (first && !rgb_image.empty() && !depth_image.empty())
      { // future work!! => Consider bounding box probability

        // set bounding box area
        box = bounding_buf.front()->bounding_boxes[0];
        bounding_buf.pop();
        long c1 = box.xmin;
        long r1 = box.ymin;
        long c2 = box.xmax;
        long r2 = box.ymax;

        //std::cout << c1 << " " << c2 << " " << r1 << " " << r2 << " " << std::endl;

        //std::cout << depth_image.at<unsigned short>(r1,c1) << std::endl;
        //std::cout << depth_image.size() << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::PointXYZRGB point;
        //Convert RGB-D to PointCloud
        for(int i = r1; i < r2; i++){
          for(int j = c1; j < c2; j++){
            // RGB-D to PointCloud2
            point.z = depth_image.at<unsigned short>(i,j) / 1000.0;
            point.x = (static_cast<float>(j)-cx) * point.z  / fx;
            point.y = (static_cast<float>(i)-cy) * point.z / fx;
            point.b = rgb_image.at<cv::Vec3b>(i,j)[0];
            point.g = rgb_image.at<cv::Vec3b>(i,j)[1];
            point.r = rgb_image.at<cv::Vec3b>(i,j)[2];
            cloud.push_back(point);

            //std::cout << point.x << "  " << point.y << "  " << point.z << std::endl;
          }
        }

        // Plane Segmenation
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        *ptr_cloud = cloud;

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        ROS_INFO("77777777777777777777777777");

        seg.setInputCloud (ptr_cloud);
        seg.segment (*inliers, *coefficients);

        pcl::toROSMsg(cloud, cloudmsg);


        // ax + by + cz + d = 0;
        std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                              << coefficients->values[1] << " "
                                              << coefficients->values[2] << " "
                                              << coefficients->values[3] << std::endl;
        //first = false;
        //ROS_INFO("Make Cloud");
      }
    }
    else // No image
    {
      ROS_INFO("Wait Images");
    }
    cloudmsg.header.frame_id = "camera";
    pointcloud_pub.publish(cloudmsg);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
