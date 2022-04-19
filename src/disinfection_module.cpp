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

  return 0;
}