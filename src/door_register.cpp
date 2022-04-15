#include "ros/ros.h"
#include <ros/package.h>

#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <opencv2/opencv.hpp>

#include "door_angle/DoorPose.h"
#include "door_angle/DoorPoses.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <iostream>
#include <vector>

std::vector<door_angle::DoorPose> vecDoor;

void doorCallback(const door_angle::DoorPosesPtr& doors)
{
  double distanceThresh = 1.0;

  for(unsigned long i = 0; i < doors->door_poses.size(); i++){
    door_angle::DoorPose doorPose = doors->door_poses[i];

    bool overlap = false;
    for(unsigned long j = 0; j < vecDoor.size(); j++){
      door_angle::DoorPose doorTmp = vecDoor[j];
      double centerX1 = static_cast<double>(doorPose.x1 + doorPose.x2) / 2.0;
      double centerY1 = static_cast<double>(doorPose.y1 + doorPose.y2) / 2.0;
      double centerX2 = static_cast<double>(doorTmp.x1  + doorTmp.x2)  / 2.0;
      double centerY2 = static_cast<double>(doorTmp.y1  + doorTmp.y2)  / 2.0;

      double distance = (centerX1 - centerX2) * (centerX1 - centerX2) + (centerY1 - centerY2) * (centerY1 - centerY2);

      //if too close
      if(distance < distanceThresh){
        overlap = true;
        break;
      }
    }
    // no overlapped
    if(!overlap){
      std::string pkg_path = ros::package::getPath("door_angle");
      std::string filePath = pkg_path + "/obj/door.yaml";

      cv::FileStorage fsOut(filePath, cv::FileStorage::APPEND);

      fsOut << "door" + std::to_string(vecDoor.size()+1);
      fsOut << "{" << "x1" << doorPose.x1
                   << "y1" << doorPose.y1
                   << "x2" << doorPose.x2
                   << "y2" << doorPose.y2 << "}";

      fsOut.release();

      vecDoor.push_back(doorPose);
    }
  }

  return;
}

int main(int argc, char **argv)
{
  //writing part need!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!￩
  ros::init(argc, argv, "door_register");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/door_poses", 1000, doorCallback);

  ros::Publisher markerArr_pub = nh.advertise<visualization_msgs::MarkerArray>("/door_marker", 1000);

  visualization_msgs::MarkerArray markerArr;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";

  std::string pkg_path = ros::package::getPath("door_angle");
  std::string filePath = pkg_path + "/obj/door.yaml";
  std::cout << filePath << std::endl;

  cv::FileStorage fsSettings(filePath, cv::FileStorage::READ);
  std::cout << fsSettings.isOpened() << std::endl;


  while(true){
    std::string cnt = "door" + std::to_string(vecDoor.size()+1);
    std::cout << cnt << std::endl;
    cv::FileNode door_pos = fsSettings[cnt];
    if(door_pos.empty()){
      break;
    }
    else{
      std::cout << "make marker" << std::endl;

      door_angle::DoorPose doorRead;

      doorRead.x1 = static_cast<float>(door_pos["x1"]);
      doorRead.y1 = static_cast<float>(door_pos["y1"]);
      doorRead.x2 = static_cast<float>(door_pos["x2"]);
      doorRead.y2 = static_cast<float>(door_pos["y2"]);

      vecDoor.push_back(doorRead);

      //double door_angle_rad = std::atan2(static_cast<double>(doorRead.y2-doorRead.y1),static_cast<double>(doorRead.x2-doorRead.x1));
      //std::cout << "door angle : " << door_angle_rad << std::endl;
    }
  }


  fsSettings.release();

  unsigned long curVecSize  = 0;
  unsigned long prevVecSize = 0;

  while (ros::ok())
  {
    if(curVecSize == prevVecSize){
      curVecSize = vecDoor.size();
    }
    else{
      for(unsigned long i = prevVecSize; i < curVecSize; i++){

        std::cout << "make marker" << std::endl;
        double x1 = static_cast<double>(vecDoor[i].x1);
        double y1 = static_cast<double>(vecDoor[i].y1);
        double x2 = static_cast<double>(vecDoor[i].x2);
        double y2 = static_cast<double>(vecDoor[i].y2);

        double door_angle_rad = std::atan2(y2-y1,x2-x1);
        std::cout << "door angle : " << door_angle_rad << std::endl;


        marker.id = static_cast<int>(i);
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

        marker.scale.x = (std::abs(x1-x2) + std::abs(y1-y2) ) * 0.5;
        marker.scale.y = 0.1;
        marker.scale.z = marker.pose.position.z * 2.0;

        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        markerArr.markers.push_back(marker);

        prevVecSize = curVecSize;
      }
    }

    markerArr_pub.publish(markerArr);
    ros::spinOnce();
  }

  return 0;
}
