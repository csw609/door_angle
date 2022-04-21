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
#include <fstream>

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

      std::ofstream fsOut(filePath, std::ios_base::app);
      //cv::FileStorage fsOut(filePath, cv::FileStorage::APPEND);
      std::string cnt = "door" + std::to_string(vecDoor.size()+1);
      fsOut << cnt + "x1: \"" << doorPose.x1 << "\"\n"
            << cnt + "y1: \"" << doorPose.y1 << "\"\n"
            << cnt + "x2: \"" << doorPose.x2 << "\"\n"
            << cnt + "y2: \"" << doorPose.y2 << "\"\n"
            << "\n";

      fsOut.close();

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
  ros::Subscriber sub = nh.subscribe("/door_poses", 10, doorCallback);

  ros::Publisher markerArr_pub = nh.advertise<visualization_msgs::MarkerArray>("/door_marker", 10);

  visualization_msgs::MarkerArray markerArr;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";

  //std::string pkg_path = ros::package::getPath("door_angle");
  //std::string filePath = pkg_path + "/obj/door.yaml";
  //std::cout << filePath << std::endl;

  //cv::FileStorage fsSettings(filePath, cv::FileStorage::READ);
  //std::cout << fsSettings.isOpened() << std::endl;



  while(true){

    std::string cnt = "door" + std::to_string(vecDoor.size()+1);
    std::cout << cnt << std::endl;
    //cv::FileNode door_pos = fsSettings[cnt];
    if(!nh.hasParam(cnt+"x1")){
      break;
    }
    else{
      std::cout << "Read " + cnt << std::endl;

      door_angle::DoorPose doorRead;
      std::string x1, x2, y1, y2;
      std::string tmp = cnt +"x1";
      nh.param<std::string>(cnt+"x1",x1,"0.0");
      nh.param<std::string>(cnt+"y1",y1,"0.0");
      nh.param<std::string>(cnt+"x2",x2,"0.0");
      nh.param<std::string>(cnt+"y2",y2,"0.0");

      std::cout << "x1 : " << x1 << "\n";
      std::cout << "y1 : " << y1 << "\n";
      std::cout << "x2 : " << x2 << "\n";
      std::cout << "y2 : " << y2 << "\n";
      doorRead.x1 = std::stof(x1);
      doorRead.y1 = std::stof(y1);
      doorRead.x2 = std::stof(x2);
      doorRead.y2 = std::stof(y2);

      vecDoor.push_back(doorRead);

      //double door_angle_rad = std::atan2(static_cast<double>(doorRead.y2-doorRead.y1),static_cast<double>(doorRead.x2-doorRead.x1));
      //std::cout << "door angle : " << door_angle_rad << std::endl;
    }
  }

  int nDoor = 1;
  while(true){

    std::string cnt = "door" + std::to_string(nDoor);

    if(!nh.hasParam(cnt+"x1")){
      break;
    }
    else{
      std::cout << "delete " + cnt << std::endl;
      nh.deleteParam(cnt+"x1");
      nh.deleteParam(cnt+"y1");
      nh.deleteParam(cnt+"x2");
      nh.deleteParam(cnt+"y2");
      nDoor++;
    }
  }

  //fsSettings.release();

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
