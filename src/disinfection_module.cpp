#include "ros/ros.h"
#include <ros/package.h>

#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>

#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>

#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <opencv2/opencv.hpp>

#include "door_angle/DoorPose.h"
#include "door_angle/DoorPoses.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <iostream>
#include <vector>

#include <door_angle/SrvDisinfect.h>

std::vector<door_angle::DoorPose> vecDoor;
/*
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server
*/
int nRobotStatus = -2;
int nDisinfStatus = 1;
std::string strMode = "nothing";

void statusCallback(const actionlib_msgs::GoalStatusArrayPtr &status)
{
  if(!status->status_list.empty()){
    nRobotStatus = static_cast<int>(status->status_list.back().status);
    //    if(strMode == "disinfection"){
    //      std::cout << nRobotStatus << "\n";
    //    }
  }
  else {
    nRobotStatus = -1;
    //std::cout << "Need Initialize?"  << "\n";
  }

}

void modeCallback(const std_msgs::StringPtr &msg)
{
  strMode = msg->data;
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "disinfection_module");
  ros::NodeHandle nh;

  // Action Server
  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Publisher
  ros::Publisher robot_pose_arr_pub  = nh.advertise<geometry_msgs::PoseArray>("/robot_pose_array", 10);
  ros::Publisher vel_pub             = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Subscriber
  ros::Subscriber robot_status_sub = nh.subscribe("/move_base/status", 10, statusCallback);
  ros::Subscriber umbot_mode_sub = nh.subscribe("/umbot_mode", 10, modeCallback);

  // Service Client
  ros::ServiceClient clDisinfect = nh.serviceClient<door_angle::SrvDisinfect>("Disinfect_service");

  // read obj
  std::string pkg_path = ros::package::getPath("door_angle");
  std::string filePath = pkg_path + "/obj/door.yaml";
  std::cout << filePath << std::endl;
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
    double dDistDoor = 0.9;

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
  bool bLocalization = true;
  int nPathCnt = 0;
  bool bGoalFlag = true;
  bool bStopFlag = true;  //true when robot stop
  std::vector<int> vecPath;

  float fPrevYError = 0.0f;
  float fPrevXError = 0.0f;
  float fPgain = 0.2f;
  float fDgain = 0.01f;
  float fYError = 0.0f;
  float fXError  = 0.0f;
  geometry_msgs::Twist msgVel;
  msgVel.linear.x  = 0.0; msgVel.linear.y  = 0.0; msgVel.linear.z  = 0.0;
  msgVel.angular.x = 0.0; msgVel.angular.y = 0.0; msgVel.angular.z = 0.0;
  while (ros::ok())
  {
    //std::cout << strMode << "\n";
    if(strMode == "disinfection"){
      bStopFlag = false;
      //std::cout << "disinfection ON" << "\n";


      try{

        tfb2m = tfBuffer.lookupTransform("map", "base_footprint",
                                         ros::Time(0));
        bLocalization = true;
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
        Tb2m(3,3) = 1.0;

        Tm2b(0,0) = rm2b(0,0); Tm2b(0,1) = rm2b(0,1); Tm2b(0,2) = rm2b(0,2);
        Tm2b(1,0) = rm2b(1,0); Tm2b(1,1) = rm2b(1,1); Tm2b(1,2) = rm2b(1,2);
        Tm2b(2,0) = rm2b(2,0); Tm2b(2,1) = rm2b(2,1); Tm2b(2,2) = rm2b(2,2);
        Tm2b(0,3) = vm2b(0);
        Tm2b(1,3) = vm2b(1);
        Tm2b(2,3) = vm2b(2);
        Tm2b(3,3) = 1.0;

        //std::cout << Tb2m << std::endl;
      }
      catch (tf2::TransformException &ex) {
        bLocalization = false;
        ROS_WARN("%s",ex.what());
        //continue;
      }

      if(bLocalization && vecPath.empty()){
        std::cout << "Make Disinfection Path" << "\n";
        Eigen::Vector4d robotPose_m;
        robotPose_m(0) = 0.0;
        robotPose_m(1) = 0.0;
        robotPose_m(2) = 0.0;
        robotPose_m(3) = 1.0;

        robotPose_m = Tb2m * robotPose_m;

        std::cout << Tb2m(0,3) << "\n";
        std::cout << Tb2m(1,3) << "\n";
        std::cout << Tb2m(2,3) << "\n";


        double dRobotX_m = robotPose_m(0);
        double dRobotY_m = robotPose_m(1);

        // calculate distance
        double table[40][40];
        bool visited[40] = {};
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
          //std::cout << dDistSQ << "\n";
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

        //        for(unsigned long i = 0; i < vecDoor.size() +1; i++){
        //          for(unsigned long j = 0; j < vecDoor.size() + 1; j++){
        //            std::cout << table[i][j] << " ";
        //          }
        //          std::cout << "\n";
        //        }

        // Make Path
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
          std::cout << "Door" << nMinIndex << "\n";
          vecPath.push_back(nMinIndex-1);
          //std::cout ;
        }
        //std::cout << "status : " << nRobotStatus << "\n";
        std::cout << "Path" <<"\n";
        for(int i = 0; i < static_cast<int>(vecPath.size()); i++){
          std::cout << vecPath[static_cast<unsigned long>(i)] + 1 << " ";
        }
        std::cout << "Path Making Complete" << "\n";
        //std::cout << "status : " << ac.getState().text_ << "\n";
        //bLocalization = false;
      }


      if(!vecPath.empty()){
        //nDisinfStatus = 1; // Debug code => later assign '1' after receive disinfection complete signal
        if((nRobotStatus == 7 || nRobotStatus == 2 || nRobotStatus == 4 || nRobotStatus == -1 || nRobotStatus == 3)
           && bGoalFlag && nDisinfStatus == 1){ // Robot Reached to Goal & Disinfected
          // loop style !
          int vecPathSize = static_cast<int>(vecPath.size());

          // once style !
          // ???

          //publish goal
          move_base_msgs::MoveBaseGoal goal;
          //          goal.header.frame_id = "map";
          unsigned long poseIndex = static_cast<unsigned long>(vecPath[static_cast<unsigned long>(nPathCnt)]);
          //          goal.goal.target_pose.pose = vecRobotPose[poseIndex].pose;
          //          goal.goal.target_pose.header.frame_id = "map";
          //          goal.goal_id.id = std::to_string(nPathCnt);
          goal.target_pose.header.frame_id = "map";
          goal.target_pose.pose = vecRobotPose[poseIndex].pose;

          ac.sendGoal(goal);
          //move_base_goal_pub.publish(goal);
          std::cout << "Move to Door" << poseIndex+1  << "\n";
          bGoalFlag = false;
          nDisinfStatus = 0;
          nPathCnt = (nPathCnt + 1) % vecPathSize;
        }
        else if(nRobotStatus == 3 && bGoalFlag && nDisinfStatus == 0){ // Robot Reached to Goal & not Disinfected
          std::cout << "Disinfecting" << "\n";

          door_angle::SrvDisinfect srv;
          srv.request.call = true;
          if(clDisinfect.call(srv)){
            ROS_INFO("Service Call Sucess");
            fYError = static_cast<float>(srv.response.YError);
            fXError  = static_cast<float>(srv.response.XError);
            float fErrorThresh = 0.05f;
            ROS_INFO("YError : %lf", static_cast<double>(fYError));
            ROS_INFO("XError : %lf", static_cast<double>(fXError));
            if(fYError < fErrorThresh){
              std::cout << "Disinfection Complete!!" << "\n";
              nDisinfStatus = 1;
              // Add check and wait
            }
            else{
              std::cout << "Disinfecting!!!!" << "\n";

              double dYVel    = static_cast<double>(fYError * fPgain + (fYError - fPrevYError) * fDgain);

              double dXVel = static_cast<double>( fXError  * fPgain + (fXError - fPrevXError) * fDgain);
              msgVel.linear.x  = 0.0;
              msgVel.linear.y  = dYVel;
              msgVel.linear.z  = 0.0;
              msgVel.angular.x = 0.0;
              msgVel.angular.y = 0.0;
              msgVel.angular.z = 0.0;
              fPrevYError       = fYError;
              fPrevXError      = fXError;
              nDisinfStatus = 0;
            }

          }
          else{
            ROS_INFO("Service Call Failed");
          }

          vel_pub.publish(msgVel);
          // add fine control? or not?

        }
        else if(nRobotStatus == 1){ // Robot is moving to door
          bGoalFlag = true;
          //std::cout << "Move to Door" << "\n";
        }
      }
    }
    else if(!bStopFlag && strMode != "stop"){

      std::cout << "Disinfection Mode Off!" << std::endl;
      std::cout << "Mode Changed!" << std::endl;
      std::cout << "Robot Stop!" << std::endl;
      //      actionlib_msgs::GoalID cancle;
      //      cancle.id = std::to_string(nPathCnt-1);
      //      move_base_cancel_pub.publish(cancle);
      ac.cancelGoal();
      nPathCnt = 0;
      bGoalFlag = true;
      bStopFlag = true;
    }
    else if(strMode != "stop"){
      nPathCnt = 0;
      bGoalFlag = true;
      //std::cout << "path clear!" << "\n";
      vecPath.clear();
      //continue;
    }
    else if(!bStopFlag && strMode == "stop"){
      std::cout << "Disinfection Mode Off!" << std::endl;
      std::cout << "Robot Stop!" << std::endl;
      ac.cancelGoal();
      int vecPathSize = static_cast<int>(vecPath.size());
      nPathCnt = ((nPathCnt - 1) + vecPathSize) % vecPathSize;
      bStopFlag = true;
    }



    robot_pose_arr_pub.publish(robotPoseArr);
    ros::spinOnce();
  }

  return 0;
}
