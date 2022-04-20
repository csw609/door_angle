#include "ros/ros.h"
#include <ros/package.h>

#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

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

    double dCenterX = (x1 + x2) * 0.5;
    double dCenterY = (y1 + y2) * 0.5;
    double dDistDoor = 0.5;

    robotPose.pose.position.x = dCenterX - std::cos(robotAngle) * dDistDoor;
    robotPose.pose.position.y = dCenterY - std::sin(robotAngle) * dDistDoor;
    robotPose.pose.position.z = 0.0;

    robotPose.header.frame_id = "map";

    vecRobotPose.push_back(robotPose);
    robotPoseArr.poses.push_back(robotPose.pose);
  }



  Eigen::Matrix4d Tb2m = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d Tm2b = Eigen::Matrix4d::Identity();

  geometry_msgs::TransformStamped tfb2m;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  bool localization = true;
  while (ros::ok())
  {

    try{

      tfb2m = tfBuffer.lookupTransform("map", "base_link",
                                       ros::Time(0));
      localization = true;
      Eigen::Quaterniond qb2m;
      qb2m.x() = tfb2m.transform.rotation.x;
      qb2m.y() = tfb2m.transform.rotation.y;
      qb2m.z() = tfb2m.transform.rotation.z;
      qb2m.w() = tfb2m.transform.rotation.w;
      Eigen::Matrix3d rb2m = qb2m.normalized().toRotationMatrix();
      Eigen::Matrix3d rm2b = rb2m.inverse();
      Eigen::Vector3d vb2m, vm2b;
      vb2m(0) = tfb2m.transform.translation.x;
      vb2m(1) = tfb2m.transform.translation.y;
      vb2m(2) = tfb2m.transform.translation.z;
      vm2b = -rm2b*vb2m;

      Tb2m(0,0) = rb2m(0,0); Tb2m(0,1) = rb2m(0,1); Tb2m(0,2) = rb2m(0,2);
      Tb2m(1,0) = rb2m(1,0); Tb2m(1,1) = rb2m(1,1); Tb2m(1,2) = rb2m(1,2);
      Tb2m(2,0) = rb2m(2,0); Tb2m(2,1) = rb2m(2,1); Tb2m(2,2) = rb2m(2,2);
      Tb2m(0,3) = tfb2m.transform.translation.x;
      Tb2m(1,3) = tfb2m.transform.translation.y;
      Tb2m(2,3) = tfb2m.transform.translation.z;

      Tm2b(0,0) = rm2b(0,0); Tm2b(0,1) = rm2b(0,1); Tm2b(0,2) = rm2b(0,2);
      Tm2b(1,0) = rm2b(1,0); Tm2b(1,1) = rm2b(1,1); Tm2b(1,2) = rm2b(1,2);
      Tm2b(2,0) = rm2b(2,0); Tm2b(2,1) = rm2b(2,1); Tm2b(2,2) = rm2b(2,2);
      Tm2b(0,3) = vm2b(0);
      Tm2b(1,3) = vm2b(1);
      Tm2b(2,3) = vm2b(2);

      //std::cout << Tb2m << std::endl;
    }
    catch (tf2::TransformException &ex) {
      //localization = false;
      //ROS_WARN("%s",ex.what());
      //continue;
    }
    if(localization){
      Eigen::Vector4d robotPose_m;
      robotPose_m.Zero();
      robotPose_m(3) = 0.0;

      robotPose_m = Tb2m * robotPose_m;

      double dRobotX_m = robotPose_m(0);
      double dRobotY_m = robotPose_m(1);


      double table[20][20];
      bool visited[20] = {};
      visited[0] = true;
      table[0][0] = 0;
      for(unsigned long i = 0; i < vecDoor.size(); i++){
        double x1 = static_cast<double>(vecDoor[i].x1);
        double y1 = static_cast<double>(vecDoor[i].y1);
        double x2 = static_cast<double>(vecDoor[i].x2);
        double y2 = static_cast<double>(vecDoor[i].y2);

        double dDoorX = (x1 + x2) * 0.5;
        double dDoorY = (y1 + y2) * 0.5;

        double dDistSQ = (dRobotX_m - dDoorX) * (dRobotX_m - dDoorX) + (dRobotY_m - dDoorY) * (dRobotY_m - dDoorY);
        std::cout << dDistSQ << "\n";
        table[0][i+1] = dDistSQ;
        table[i+1][0] = dDistSQ;
      }
      for(unsigned long i = 0; i < vecDoor.size(); i++){
        for(unsigned long j = 0; j < vecDoor.size(); j++){
          if(i == j) {
            table[i+1][j+1] = 0.0;
            continue;
          }
          double x1 = static_cast<double>(vecDoor[i].x1);
          double y1 = static_cast<double>(vecDoor[i].y1);
          double x2 = static_cast<double>(vecDoor[i].x2);
          double y2 = static_cast<double>(vecDoor[i].y2);

          double dDoorXi = (x1 + x2) * 0.5;
          double dDoorYi = (y1 + y2) * 0.5;

          x1 = static_cast<double>(vecDoor[j].x1);
          y1 = static_cast<double>(vecDoor[j].y1);
          x2 = static_cast<double>(vecDoor[j].x2);
          y2 = static_cast<double>(vecDoor[j].y2);

          double dDoorXj = (x1 + x2) * 0.5;
          double dDoorYj = (y1 + y2) * 0.5;

          double dDistSQ = (dDoorXi - dDoorXj) * (dDoorXi - dDoorXj) + (dDoorYi - dDoorYj) * (dDoorYi - dDoorYj);

          table[i+1][j+1] = dDistSQ;
          table[j+1][i+1] = dDistSQ;
        }
      }

      for(unsigned long i = 0; i < vecDoor.size() +1; i++){
        for(unsigned long j = 0; j < vecDoor.size() + 1; j++){
          std::cout << table[i][j] << " ";
        }
        std::cout << "\n";

      }
      int startIndex = 0;
      for(unsigned long i = 0; i < vecDoor.size(); i++){
        double dMin = 987654321.0;
        int nMinIndex = 0;
        for(int j = 0; j < static_cast<int>(vecDoor.size()) + 1; j++){
          if(startIndex == j) continue;
          if(visited[j]) continue;
          if(dMin > table[startIndex][j]){
            dMin = table[startIndex][j];
            nMinIndex = j;
          }
        }
        visited[nMinIndex] = true;
        startIndex = nMinIndex;
        std::cout << nMinIndex << "\n";
        //std::cout ;

      }



      localization = false;

    }

    robot_pose_arr_pub.publish(robotPoseArr);
    ros::spinOnce();
  }

  return 0;
}
