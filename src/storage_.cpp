#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


void door_write(){
  std::string pkg_path = ros::package::getPath("door_angle");
  std::string filePath = pkg_path + "/obj/door.yaml";

  cv::FileStorage fsOut(filePath, cv::FileStorage::APPEND);
  int a = 2;
  fsOut << "door" + std::to_string(a);
  fsOut << "{" << "x1" << 4.39045000076
               << "y1" << 2
               << "x2" << 3
               << "y2" << 4 << "}";

  fsOut.release();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "storage");
  ros::NodeHandle nh;

  ros::Publisher markerArr_pub = nh.advertise<visualization_msgs::MarkerArray>("/door_marker", 1000);
  visualization_msgs::MarkerArray markerArr;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";


  std::string pkg_path = ros::package::getPath("door_angle");
  std::string filePath = pkg_path + "/obj/door.yaml";
  std::cout << filePath << std::endl;

  cv::FileStorage fsSettings(filePath, cv::FileStorage::READ);
  std::cout << fsSettings.isOpened() << std::endl;
  int count = 1;
  while(true){
    std::string cnt = "door" + std::to_string(count);
    std::cout << cnt << std::endl;
    cv::FileNode door_pos = fsSettings[cnt];
    if(door_pos.empty()){
      break;
    }
    else{
      std::cout << "make marker" << std::endl;
      double x1 = static_cast<double>(door_pos["x1"]);
      double y1 = static_cast<double>(door_pos["y1"]);
      double x2 = static_cast<double>(door_pos["x2"]);
      double y2 = static_cast<double>(door_pos["y2"]);

      double door_angle_rad = std::atan2(y2-y1,x2-x1);
      std::cout << "door angle : " << door_angle_rad << std::endl;


      marker.id = count;
      marker.header.stamp = ros::Time::now();
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = (x1 + x2) / 2.0;
      marker.pose.position.y = (y1 + y2) / 2.0;
      marker.pose.position.z = 0.75;
      Eigen::Quaterniond q;
      q = Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(door_angle_rad,Eigen::Vector3d::UnitZ());
      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.pose.orientation.w = q.w();

      marker.scale.x = std::abs(x1-x2);
      marker.scale.y = 0.1;
      marker.scale.z = marker.pose.position.z * 2.0;

      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;

      markerArr.markers.push_back(marker);
      count++;
    }
  }


  fsSettings.release();


  ros::Rate loop_rate(10);



  while (ros::ok())
  {
    markerArr_pub.publish(markerArr);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
