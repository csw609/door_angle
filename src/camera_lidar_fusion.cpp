#include "ros/ros.h"
#include "std_msgs/String.h"
#include <queue>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "door_angle/BoundingBox.h"
#include "door_angle/BoundingBoxes.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

std::queue<sensor_msgs::ImageConstPtr>         image_buf;
std::queue<sensor_msgs::LaserScanConstPtr>     scan_buf;
std::queue<door_angle::BoundingBoxesPtr>       bounding_buf;

void imgCallback(const sensor_msgs::ImageConstPtr &image)
{
  image_buf.push(image);
}

void scanCallback(const sensor_msgs::LaserScanConstPtr &scan)
{
  scan_buf.push(scan);
}

void boundCallback(const door_angle::BoundingBoxesPtr &boxes)
{
  bounding_buf.push(boxes);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_lidar_fusion");
  ros::NodeHandle nh;

  // Pub & Sub
  ros::Publisher fusion_image_pub = nh.advertise<sensor_msgs::Image>("fusion_image", 1000);

  ros::Subscriber image_sub = nh.subscribe("/camera/color/image_raw", 1000, imgCallback);
  ros::Subscriber bouding_box_sub = nh.subscribe("/bounding_box_array",1000, boundCallback);
  ros::Subscriber scan_sub = nh.subscribe("/scan", 1000, scanCallback);

  // Transformation matrix
  Eigen::Matrix4d Tl2b = Eigen::Matrix4d::Identity();
  //Eigen::Matrix4d Tc2b = Eigen::Matrix4d::Identity();
  //Eigen::Matrix4d Tb2l = Eigen::Matrix4d::Identity();

  Eigen::Matrix4d Tb2c = Eigen::Matrix4d::Zero();
  Tb2c(2,0) = 1;
  Tb2c(0,1) = -1;
  Tb2c(1,2) = -1;

  // Camera Intrinsic
  Eigen::Matrix3d intrinsic;
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
      intrinsic(i,j) = 0;
    }
  }
  intrinsic(0,0) = 610.8421650074351;
  intrinsic(1,1) = 610.0254467507624;
  intrinsic(0,2) = 336.7626691266633;
  intrinsic(1,2) = 252.1813169593813;
  intrinsic(2,2) = 1.0;

  cv_bridge::CvImage cv_bridge;

  ros::Rate loop_rate(1000);

  int lack_count = 0;

  while (ros::ok())
  {
    // If data received
    if (!image_buf.empty() && !scan_buf.empty() && !bounding_buf.empty())
    {
      //ROS_INFO("buf no empty");
      lack_count = 0;

      double time;
      double time_r = image_buf.front()->header.stamp.toSec();
      double time_s = scan_buf.front()->header.stamp.toSec();
      double time_b = bounding_buf.front()->header.stamp.toSec();

      cv::Mat image;
      cv::Mat depth_image;

      sensor_msgs::LaserScan scan;
      door_angle::BoundingBoxes boxes;
      std_msgs::Header header;



      //rgb, scan, bounding box time sync 0.003
      if (time_r < time_s - 0.003)
      {
        image_buf.pop();
        ROS_INFO("pop rgb_image\n");
      }
      else if (time_r > time_s + 0.003)
      {
        scan_buf.pop();
        ROS_INFO("pop scan\n");
      }
      else if(time_r < time_b - 0.003){
        image_buf.pop();
        ROS_INFO("pop rgb_image\n");
      }
      else if(time_r > time_b + 0.003){
        bounding_buf.pop();
        ROS_INFO("pop bound header\n");
      }
      else if(!bounding_buf.front()->bounding_boxes.empty())  //bounding box not empty && satisfy time sync
      {
        time = image_buf.front()->header.stamp.toSec();

        //convert sensor_msgs::image to cv::Mat
        cv_bridge::CvImageConstPtr ptr_l;
        ptr_l = cv_bridge::toCvCopy(image_buf.front(), sensor_msgs::image_encodings::BGR8);

        image = ptr_l->image.clone();
        header = image_buf.front()->header;
        image_buf.pop();

        // scan copy
        scan.ranges = scan_buf.front()->ranges;
        scan.header = scan_buf.front()->header;
        scan.angle_max = scan_buf.front()->angle_max;
        scan.angle_min = scan_buf.front()->angle_min;
        scan.range_max = scan_buf.front()->range_max;
        scan.range_min = scan_buf.front()->range_min;
        scan.scan_time = scan_buf.front()->scan_time;
        scan.intensities = scan_buf.front()->intensities;
        scan.time_increment = scan_buf.front()->time_increment;
        scan.angle_increment = scan_buf.front()->angle_increment;
        scan_buf.pop();
      }
      if(!image.empty() && !scan.ranges.empty()){
        //ROS_INFO("debug11111");
        boxes.bounding_boxes = bounding_buf.front()->bounding_boxes;
        std::cout << " number of bounding boxes : " << boxes.bounding_boxes.size();

        std::vector<Eigen::Vector3d> scanPoints;
        std::vector<Eigen::Vector3d> s2iPoints;
        double min_angle = static_cast<double>(scan.angle_min);
        double diff_angle = static_cast<double>(scan.angle_increment);


        // image lidar fusion
        for(unsigned long i = 0; i < scan.ranges.size(); i++){

          // get lidar point
          Eigen::Vector4d point_l;
          point_l(0) = std::cos(min_angle + diff_angle * i) * static_cast<double>(scan.ranges[i]);
          point_l(1) = std::sin(min_angle + diff_angle * i) * static_cast<double>(scan.ranges[i]);
          point_l(2) = 0.0; // z
          point_l(3) = 1.0;

          //scanPoints.push_back(point);
          // frame convert
          // lidar => base_link frame => camera
          Eigen::Vector4d point_c;
          point_c = Tb2c * Tl2b * point_l;

          Eigen::Vector3d point_c3;
          point_c3(0) = point_c(0);
          point_c3(1) = point_c(1);
          point_c3(2) = point_c(2);

          Eigen::Vector3d point_uv;

          // camera => image plane
          point_uv = intrinsic * point_c3;

          point_uv(0) = point_uv(0) /  point_uv(2);
          point_uv(1) = point_uv(1) / point_uv(2);
          point_uv(2) = point_uv(2) / point_uv(2);

          // draw lidar point on image
          int r = static_cast<int>(point_uv(1));
          int c = static_cast<int>(point_uv(0));
          if(r > 0 && r < image.rows && c > 0 && c < image.cols){
            image.at<cv::Vec3b>(r,c)[0] = 255;
            image.at<cv::Vec3b>(static_cast<int>(point_uv(1)),static_cast<int>(point_uv(0)))[1] = 255;
            image.at<cv::Vec3b>(static_cast<int>(point_uv(1)),static_cast<int>(point_uv(0)))[2] = 255;
          }

          s2iPoints.push_back(point_uv);
        }

        sensor_msgs::Image fusion_image;
        cv_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
        cv_bridge.toImageMsg(fusion_image);

        // bouding box fusion process
        for(unsigned long i = 0; i < boxes.bounding_boxes.size(); i++){
          door_angle::BoundingBox box;
          box = boxes.bounding_boxes[0];
          if( box.Class == "door"){
            continue;
          }
          else if( box.Class == "handle"){
            std::cout << "handle!!!!" << std::endl;
          }
          else{
            continue;
          }
        }

        //publish fusion image
        fusion_image_pub.publish(fusion_image);

      }
    }

    else{
      lack_count++;
      if(lack_count % 2000 == 0){
        ROS_INFO("wait messages");

        lack_count = 0;
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  } //while

  return 0;
}
