#include "ros/ros.h"
#include "std_msgs/String.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry odometry;
bool bOdom = false;

void odomCallback(const nav_msgs::OdometryPtr &odom){
  //Eigen::Matrix4d Tb2x;
  Eigen::Quaterniond qb2x;
  qb2x.x() = odom->pose.pose.orientation.x;
  qb2x.y() = odom->pose.pose.orientation.y;
  qb2x.z() = odom->pose.pose.orientation.z;
  qb2x.w() = odom->pose.pose.orientation.w;

  Eigen::Matrix3d rb2x = qb2x.normalized().toRotationMatrix();

//    Tb2x(0,0) = rb2x(0,0); Tb2x(0,1) = rb2x(0,1); Tb2x(0,2) = rb2x(0,2);
//    Tb2x(1,0) = rb2x(1,0); Tb2x(1,1) = rb2x(1,1); Tb2x(1,2) = rb2x(1,2);
//    Tb2x(2,0) = rb2x(2,0); Tb2x(2,1) = rb2x(2,1); Tb2x(2,2) = rb2x(2,2);
//    Tb2x(0,3) = odom->pose.pose.position.x;
//    Tb2x(1,3) = odom->pose.pose.position.y;
//    Tb2x(2,3) = odom->pose.pose.position.z;
//    Tb2x(3,3) = 1.0;

  //Eigen::Matrix4d Tx2o;
  Eigen::Quaterniond qx2o;
  qx2o.x() =  0.299;
  qx2o.y() = -0.299;
  qx2o.z() =  0.641;
  qx2o.w() =  0.641;

  Eigen::Matrix3d rx2o = qx2o.normalized().toRotationMatrix();

//    Tx2o(0,0) = rx2o(0,0); Tx2o(0,1) = rx2o(0,1); Tx2o(0,2) = rx2o(0,2);
//    Tx2o(1,0) = rx2o(1,0); Tx2o(1,1) = rx2o(1,1); Tx2o(1,2) = rx2o(1,2);
//    Tx2o(2,0) = rx2o(2,0); Tx2o(2,1) = rx2o(2,1); Tx2o(2,2) = rx2o(2,2);
//    Tx2o(0,3) = 0.0;
//    Tx2o(1,3) = 0.0;
//    Tx2o(2,3) = 0.0;
//    Tx2o(3,3) = 1.0;

  Eigen::Matrix3d rb2o =  rb2x * rx2o;
  //Eigen::Matrix4d Tb2o = Tx2o * Tb2x;

  Eigen::Quaterniond qb2o(rb2o);

  Eigen::Vector3d vLinearVel;
  Eigen::Vector3d vAngularVel;
  vLinearVel(0)= odom->twist.twist.linear.x;
  vLinearVel(1)= odom->twist.twist.linear.y;
  vLinearVel(2)= odom->twist.twist.linear.z;

  vAngularVel(0)= odom->twist.twist.angular.x;
  vAngularVel(1)= odom->twist.twist.angular.y;
  vAngularVel(2)= odom->twist.twist.angular.z;

  vLinearVel = rb2o * vLinearVel;
  vAngularVel = rb2o * vAngularVel;

//  std::cout << vLinearVel(0) << std::endl;
//  std::cout << vLinearVel(1) << std::endl;
//  std::cout << vLinearVel(2) << std::endl;
//  std::cout << rb2o(0,0) << " " << rb2o(0,1) << " " << rb2o(0,2) << std::endl;
//  std::cout << rb2o(1,0) << " " << rb2o(1,1) << " " << rb2o(1,2) << std::endl;
//  std::cout << rb2o(2,0) << " " << rb2o(2,1) << " " << rb2o(2,2) << std::endl;

  odometry.header.frame_id = "odom";
  odometry.child_frame_id = "base_footprint";

  odometry.pose.pose.position.x = odom->pose.pose.position.x;
  odometry.pose.pose.position.y = odom->pose.pose.position.y;
  odometry.pose.pose.position.z = 0.0;
  odometry.pose.pose.orientation.x = qb2o.x();
  odometry.pose.pose.orientation.y = qb2o.y();
  odometry.pose.pose.orientation.z = qb2o.z();
  odometry.pose.pose.orientation.w = qb2o.w();

  odometry.twist.twist.linear.x = vLinearVel(0);
  odometry.twist.twist.linear.y = vLinearVel(1);
  odometry.twist.twist.linear.z = vLinearVel(2);

  odometry.twist.twist.angular.x = vAngularVel(0);
  odometry.twist.twist.angular.y = vAngularVel(1);
  odometry.twist.twist.angular.z = vAngularVel(2);

  bOdom = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_tf");
  ros::NodeHandle nh;

  ros::Subscriber odom_sub = nh.subscribe("/vins_estimator/odometry", 10, odomCallback);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

  while(ros::ok()){

    if(bOdom){
      odom_pub.publish(odometry);
      bOdom = false;
    }

    ros::spinOnce();
  }

  return 0;
}
