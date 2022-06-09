#include <queue>
#include <stack>
#include <vector>
#include <algorithm>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"

#include "eigen3/Eigen/Core"

#include "door_angle/SrvDisinfect.h"
#include "door_angle/BoundingBox.h"
#include "door_angle/BoundingBoxes.h"

//std::stack<sensor_msgs::ImageConstPtr>         image_buf;
std::stack<sensor_msgs::LaserScanConstPtr>     scan_buf;
std::stack<door_angle::BoundingBoxesPtr>       bounding_buf;



// parameters
double dMinDist;
double dProbThresh;
double dCx;
double dFx;

double dLaserCameraDist = 0.225;

//void imgCallback(const sensor_msgs::ImageConstPtr &image)
//{
//  if(image_buf.size() > 20){
//    while(!image_buf.empty()){
//      image_buf.pop();
//    }
//  }

//  image_buf.push(image);
//}

void scanCallback(const sensor_msgs::LaserScanConstPtr &scan)
{
  if(scan_buf.size() > 20){
    while(!scan_buf.empty()){
      scan_buf.pop();
    }
  }

  scan_buf.push(scan);
}

void boundCallback(const door_angle::BoundingBoxesPtr &boxes)
{
  if(bounding_buf.size() > 20){
    while(!bounding_buf.empty()){
      bounding_buf.pop();
    }
  }

  bounding_buf.push(boxes);
}

// camera intrinsic!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// align with door, align with handle!!!!
bool disinfect(door_angle::SrvDisinfect::Request  &req,
               door_angle::SrvDisinfect::Response &res)
{
  if(!scan_buf.empty() && !bounding_buf.empty()){
    // Read last data
    sensor_msgs::LaserScan msgScan = (*scan_buf.top());
    //sensor_msgs::Image msgImage = (*image_buf.top());
    door_angle::BoundingBoxes msgBoxes = (*bounding_buf.top());


    // Clear buffer
    while(!scan_buf.empty()){
      scan_buf.pop();
    }
//    while(!image_buf.empty()){
//      image_buf.pop();
//    }
    while(!bounding_buf.empty()){
      bounding_buf.pop();
    }

    // Scan process => convert scan ranges to 4d points

    //std::vector<Eigen::Vector4d> vecScanPoints;
    double dMinAngle  = static_cast<double>(msgScan.angle_min);
    double dDiffAngle = static_cast<double>(msgScan.angle_increment);

      // Convert scan data msgScan => std::vector<Eigen::Vector4d>
    unsigned long nScanSize = msgScan.ranges.size();
    std::vector<double> vecDistFromDoor;
    int    nDistFromDoorCnt = 0;
    for(unsigned long i =0; i < nScanSize; i++){
      if( static_cast<double>(msgScan.ranges[i]) < dMinDist) continue; // too close scan data assumed as robot frame so throw them

      double dAngle = dMinAngle + dDiffAngle * static_cast<double>(i);

      //if(dAngle < -1.57079632675 || dAngle > 1.57079632675) continue; // throw half of scan from -x axis side

      //scan points for measure distance from door
      if(dAngle < 0.436332 && dAngle > - 0.436332){   // +- 25 Deg

        double dDist;
        dDist = std::cos(dAngle) * static_cast<double>(msgScan.ranges[i]);
        vecDistFromDoor.push_back(dDist);
        ROS_INFO("dist : %lf", dDist);


        //nDistFromDoorCnt++;
      }

      //ranges to 4d
      //Eigen::Vector4d vScanPoint;
      //vScanPoint(0) = std::cos(dAngle) * static_cast<double>(msgScan.ranges[i]);
      //vScanPoint(0) = std::sin(dAngle) * static_cast<double>(msgScan.ranges[i]);
      //vScanPoint(0) = 0.0;
      //vScanPoint(0) = 1.0; // Homogeneous

      //vecScanPoints.push_back(vScanPoint);
    }

    sort(vecDistFromDoor.begin(), vecDistFromDoor.end());

    int nQ1Idx = static_cast<int>(vecDistFromDoor.size()) / 4;
    int nQ3Idx = static_cast<int>(vecDistFromDoor.size()) / 4 * 3;

    double dQ1 = vecDistFromDoor[static_cast<unsigned long >(nQ1Idx)];
    double dQ3 = vecDistFromDoor[static_cast<unsigned long >(nQ3Idx)];
    double dIQR = dQ3 - dQ1;

    double dOutStep = 1.5 * dIQR;

    double dLowerBound = dQ1 - dOutStep;
    double dUpperBound = dQ3 + dOutStep;

    unsigned long ulVecSize = vecDistFromDoor.size();

    double dDistSum = 0.0;
    int    nDistCnt = 0;
    for(unsigned long i = 0; i < ulVecSize; i++){
      if(vecDistFromDoor[i] < dUpperBound && vecDistFromDoor[i] > dLowerBound){
        ROS_INFO("dist inlier : %lf", vecDistFromDoor[i]);
        dDistSum += vecDistFromDoor[i];
        nDistCnt++;
      }
    }
    ROS_INFO("dist sum : %lf ", dDistSum);

    double dDistFromDoor;
    if(nDistCnt > 0){
      dDistFromDoor = dDistSum / static_cast<double>(nDistCnt);
    }
    else{
      ROS_INFO("Measure Distance Fail!!!!!");

      return false;
    }
    // after change to min !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // Bounding Box Process => get door handle position

    unsigned long nBoxesSize = msgBoxes.bounding_boxes.size();

    double dMinHandleX  = 0.0;
    int    nHandleCount = 0;
    for(unsigned long i = 0; i < nBoxesSize; i++){
      door_angle::BoundingBox msgBox;
      msgBox = msgBoxes.bounding_boxes[i];

      if(msgBox.probability < dProbThresh) continue; // ignore bounding box that has low probability
      //if(msgBox.Class == "handle") vecHandleBox.push_back(msgBox);
      //if(msgBox.Class == "door") vecDoorBox.push_back(msgBox);
      if(msgBox.Class == "handle"){
        dMinHandleX += msgBox.xmin;
        dMinHandleX += msgBox.xmax;
        nHandleCount += 2;
      }
    }

    float fDist2Door = 0.40f;
    //nHandleCount = 0; //debug code
    if(nHandleCount > 0){
      dMinHandleX = dMinHandleX / static_cast<double>(nHandleCount);
    }
    else{
      ROS_INFO("Handle doesn't detected!!");
      res.YError = 123123.123f;
      res.XError = static_cast<float>(dDistFromDoor - static_cast<double>(fDist2Door));
      return true;
    }

    double dHandleAngleFromXAxis = std::atan2(dFx ,(dMinHandleX - dCx)) - 1.57079632675;
    double dDistDoorCamera       = dDistFromDoor - dLaserCameraDist;
    ROS_INFO("HandleX : %lf", dMinHandleX);
    ROS_INFO("angle : %lf", dHandleAngleFromXAxis);
    ROS_INFO("Dist : %lf", dDistFromDoor);
    double dHandleYLaser         =  std::tan(dHandleAngleFromXAxis) * dDistDoorCamera; // ( - ) camera  -X Axis  == laser Y Axis

    res.YError = static_cast<float>(dHandleYLaser);

    res.XError = static_cast<float>(dDistFromDoor - static_cast<double>(fDist2Door));

    // float double check!!!!!!!!!!!!!!!!!!
    return true;

  }

  ROS_INFO("response fail");
  return false;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "disinfection_srv");
  ros::NodeHandle nh;

  // read parameter
  std::string image_topic, bounding_topic, scan_topic;
  std::string fx,fy,cx,cy, sync_tol, ransac_iter, ransac_thr, minDist, probThresh;
  //nh.param<std::string>("image_topic",image_topic,"/camera/color/image_raw");
  nh.param<std::string>("bounding_topic",bounding_topic,"/bounding_box_array");
  nh.param<std::string>("scan_topic",scan_topic,"/scan");

  nh.param<std::string>("fx",fx,"462.1379699707031");
  nh.param<std::string>("fy",fy,"462.1379699707031");
  nh.param<std::string>("cx",cx,"320.0");
  nh.param<std::string>("cy",cy,"240.0");
  nh.param<std::string>("sync_tolerance",sync_tol,"0.1");
  nh.param<std::string>("ransac_iteration",ransac_iter,"60");
  nh.param<std::string>("ransac_thershold",ransac_thr,"0.1");
  nh.param<std::string>("min_Dist", minDist, "0.35");
  nh.param<std::string>("prob_thresh", probThresh, "0.6");


  ROS_INFO("image topic : %s", image_topic.c_str());
  ROS_INFO("bounding boxes topic : %s", bounding_topic.c_str());
  ROS_INFO("lidar scan topic: %s", scan_topic.c_str());

  ROS_INFO("fx : %s", fx.c_str());
  ROS_INFO("fy : %s", fy.c_str());
  ROS_INFO("cx : %s", cx.c_str());
  ROS_INFO("cy : %s", cy.c_str());

  ROS_INFO("sync_tolernace : %s", sync_tol.c_str());
  ROS_INFO("ransac iteration : %s", ransac_iter.c_str());
  ROS_INFO("ransac threshold : %s", ransac_thr.c_str());
  ROS_INFO("lidar min distance : %s", minDist.c_str());
  ROS_INFO("Probability Threshold : %s", probThresh.c_str());


  dFx         = std::stod(fx);
  dCx         = std::stod(cx);
  dMinDist    = std::stod(minDist);
  dProbThresh = std::stod(probThresh);

  // service
  ros::ServiceServer service = nh.advertiseService("Disinfect_service", disinfect);
  ROS_INFO("Ready to Disinfection Server");

  // subscriber
  //ros::Subscriber image_sub = nh.subscribe(image_topic, 10, imgCallback);
  ros::Subscriber bouding_box_sub = nh.subscribe(bounding_topic,10, boundCallback);
  ros::Subscriber scan_sub = nh.subscribe(scan_topic, 10, scanCallback);



  while(ros::ok()){

    ros::spinOnce();
  }

  return 0;
}

