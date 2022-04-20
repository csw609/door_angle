#include "ros/ros.h"
#include <ros/package.h>

#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <opencv2/opencv.hpp>

#include "door_angle/DoorPose.h"
#include "door_angle/DoorPoses.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <iostream>
#include <vector>

std::vector<door_angle::DoorPose> vecDoor;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "disinfection_module");
  ros::NodeHandle nh;

  ros::Publisher robot_pose_arr_pub = nh.advertise<geometry_msgs::PoseArray>("/robot_pose_array", 1000);

  std::string pkg_path = ros::package::getPath("door_angle");
  std::string filePath = pkg_path + "/obj/door.yaml";
  std::cout << filePath << std::endl;

  //cv::FileStorage fsSettings(filePath, cv::FileStorage::READ);
  //std::cout << fsSettings.isOpened() << std::endl;

  while(true){

    std::string cnt = "door" + std::to_string(vecDoor.size()+1);
    //std::cout << cnt << std::endl;
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
  std::vector<geometry_msgs::PoseStamped> vecRobotPose;
  geometry_msgs::PoseArray robotPoseArr;
  robotPoseArr.header.frame_id = "map";
  geometry_msgs::PoseStamped robotPose;
  for(unsigned long i = 0; i < vecDoor.size(); i++){
    double x1 = static_cast<double>(vecDoor[i].x1);
    double y1 = static_cast<double>(vecDoor[i].y1);
    double x2 = static_cast<double>(vecDoor[i].x2);
    double y2 = static_cast<double>(vecDoor[i].y2);

    double door_angle_rad = std::atan2(y2-y1,x2-x1);
    //std::cout << "door angle : " << door_angle_rad << std::endl;

    double robotAngle = door_angle_rad + 3.1415926535 * 0.5;

    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(robotAngle,Eigen::Vector3d::UnitZ());

    robotPose.pose.orientation.x = q.x();
    robotPose.pose.orientation.y = q.y();
    robotPose.pose.orientation.z = q.z();
    robotPose.pose.orientation.w = q.w();

    double dCenterX = (x1 + x2) / 2.0;
    double dCenterY = (y1 + y2) / 2.0;
    double dDistDoor = 0.5;

    robotPose.pose.position.x = dCenterX - std::cos(robotAngle) * dDistDoor;
    robotPose.pose.position.y = dCenterY - std::sin(robotAngle) * dDistDoor;
    robotPose.pose.position.z = 0.0;

    robotPose.header.frame_id = "map";

    vecRobotPose.push_back(robotPose);
    robotPoseArr.poses.push_back(robotPose.pose);
  }

  while (ros::ok())
  {
    //TSP using
    robot_pose_arr_pub.publish(robotPoseArr);
    ros::spinOnce();
  }

  while(true){

    std::string cnt = "door" + std::to_string(vecDoor.size()+1);
    //std::cout << cnt << std::endl;
    //cv::FileNode door_pos = fsSettings[cnt];
    if(!nh.hasParam(cnt+"x1")){
      break;
    }
    else{
      std::cout << "delete " + cnt << std::endl;
      nh.deleteParam(cnt);
      //double door_angle_rad = std::atan2(static_cast<double>(doorRead.y2-doorRead.y1),static_cast<double>(doorRead.x2-doorRead.x1));
      //std::cout << "door angle : " << door_angle_rad << std::endl;
    }
  }


  return 0;
}
