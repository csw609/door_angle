#include "ros/ros.h"
#include <ros/package.h>

#include "std_msgs/String.h"

#include <opencv2/opencv.hpp>

#include "door_angle/DoorPose.h"
#include "door_angle/DoorPoses.h"

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
      vecDoor.push_back(doorPose);
    }

  }
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "door_register");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/door_poses", 1000, doorCallback);

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

      door_angle::DoorPose doorRead;

      doorRead.x1 = static_cast<float>(door_pos["x1"]);
      doorRead.y1 = static_cast<float>(door_pos["y1"]);
      doorRead.x2 = static_cast<float>(door_pos["x2"]);
      doorRead.y2 = static_cast<float>(door_pos["y2"]);

      vecDoor.push_back(doorRead);

      //double door_angle_rad = std::atan2(static_cast<double>(doorRead.y2-doorRead.y1),static_cast<double>(doorRead.x2-doorRead.x1));
      //std::cout << "door angle : " << door_angle_rad << std::endl;
      count++;
    }
  }


  fsSettings.release();


  ros::spin();

  return 0;
}
