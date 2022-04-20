#include "ros/ros.h"
#include "std_msgs/String.h"
#include <queue>
#include <deque>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "door_angle/BoundingBox.h"
#include "door_angle/BoundingBoxes.h"
#include "door_angle/DoorPose.h"
#include "door_angle/DoorPoses.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <time.h>
#include <stdlib.h>

#include <time/choi_time.h>

std::queue<sensor_msgs::ImageConstPtr>         image_buf;
std::queue<sensor_msgs::LaserScanConstPtr>     scan_buf;
std::queue<door_angle::BoundingBoxesPtr>       bounding_buf;

void imgCallback(const sensor_msgs::ImageConstPtr &image)
{
  image_buf.push(image);
}

void scanCallback(const sensor_msgs::LaserScanConstPtr &scan)
{
  scan_buf.push(scan);
}

void boundCallback(const door_angle::BoundingBoxesPtr &boxes)
{
  bounding_buf.push(boxes);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_lidar_fusion");
  ros::NodeHandle nh;

  // parameters
  std::string image_topic, bounding_topic, scan_topic;
  std::string fx,fy,cx,cy;

  nh.param<std::string>("image_topic",image_topic,"/camera/color/image_raw");
  nh.param<std::string>("bounding_topic",bounding_topic,"/bounding_box_array");
  nh.param<std::string>("scan_topic",scan_topic,"/scan");

  nh.param<std::string>("fx",fx,"462.1379699707031");
  nh.param<std::string>("fy",fy,"462.1379699707031");
  nh.param<std::string>("cx",cx,"320.0");
  nh.param<std::string>("cy",cy,"240.0");

  ROS_INFO("image topic : %s", image_topic.c_str());
  ROS_INFO("bounding boxes topic : %s", bounding_topic.c_str());
  ROS_INFO("lidar scan topic: %s", scan_topic.c_str());

  ROS_INFO("fx : %s", fx.c_str());
  ROS_INFO("fy : %s", fy.c_str());
  ROS_INFO("cx : %s", cx.c_str());
  ROS_INFO("cy : %s", cy.c_str());

  // Pub & Sub
  ros::Publisher fusion_image_pub = nh.advertise<sensor_msgs::Image>("/fusion_image", 1000);
  ros::Publisher cloud_door_pub   = nh.advertise<sensor_msgs::PointCloud2>("/cloud_door", 1000);
  ros::Publisher door_poses_pub   = nh.advertise<door_angle::DoorPoses>("/door_poses", 1000);

  ros::Subscriber image_sub = nh.subscribe(image_topic, 1000, imgCallback);
  ros::Subscriber bouding_box_sub = nh.subscribe(bounding_topic,1000, boundCallback);
  ros::Subscriber scan_sub = nh.subscribe(scan_topic, 1000, scanCallback);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Transformation matrix
  Eigen::Matrix4d Tl2b = Eigen::Matrix4d::Identity();
  //Tl2b(2,3) = 0.0;

  Eigen::Matrix4d Tb2l = Eigen::Matrix4d::Identity();
  //Tb2l(2,3) = -Tl2b(2,3);

  Eigen::Matrix4d Tb2c = Eigen::Matrix4d::Zero();
  Tb2c(2,0) = 1.0;
  Tb2c(0,1) = -1.0;
  Tb2c(1,2) = -1.0;
  Tb2c(3,3) = 1.0;

  Eigen::Matrix4d Tc2b = Eigen::Matrix4d::Zero();
  // Tranpose Tb2c's rotation part => if they get translation code should be modify
  Tc2b(0,2) = Tb2c(2,0);
  Tc2b(1,0) = Tb2c(0,1);
  Tc2b(2,1) = Tb2c(1,2);
  Tc2b(3,3) = 1.0;

  Eigen::Matrix4d Tb2m = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d Tm2b = Eigen::Matrix4d::Identity();

  // Camera Intrinsic
  Eigen::Matrix3d intrinsic;
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
      intrinsic(i,j) = 0;
    }
  }
  intrinsic(0,0) = std::stod(fx);
  intrinsic(1,1) = std::stod(fy);
  intrinsic(0,2) = std::stod(cx);
  intrinsic(1,2) = std::stod(cy);
  intrinsic(2,2) = 1.0;

  // cv
  cv_bridge::CvImage cv_bridge;

  //ros::Rate loop_rate(1000);

  int lack_count = 0;
  sensor_msgs::PointCloud2 cloudmsg;

  geometry_msgs::TransformStamped tfs2b;
  geometry_msgs::TransformStamped tfc2b;
  geometry_msgs::TransformStamped tfb2m;

  bool tf_received = false;

  while (ros::ok())
  {

    //receive transform
    if(!tf_received){

      try{
        tfs2b = tfBuffer.lookupTransform("base_link", "velodyne",
                                         ros::Time(0));
        tfc2b = tfBuffer.lookupTransform("base_link", "d435_color_optical_frame",
                                         ros::Time(0));
        ROS_INFO("%s", tfs2b.child_frame_id.c_str());
        ROS_INFO("%s", tfc2b.child_frame_id.c_str());
        //ROS_INFO("%f" ,tfs2b.transform.translation.z);

        Eigen::Quaterniond qs2b;
        qs2b.x() = tfs2b.transform.rotation.x;
        qs2b.y() = tfs2b.transform.rotation.y;
        qs2b.z() = tfs2b.transform.rotation.z;
        qs2b.w() = tfs2b.transform.rotation.w;
        Eigen::Matrix3d rs2b = qs2b.normalized().toRotationMatrix();
        Eigen::Matrix3d rb2s = rs2b.inverse();
        Eigen::Vector3d vs2b, vb2s;
        vs2b(0) = tfs2b.transform.translation.x;
        vs2b(1) = tfs2b.transform.translation.y;
        vs2b(2) = tfs2b.transform.translation.z;
        vb2s = -rb2s*vs2b;

        Tl2b(0,0) = rs2b(0,0); Tl2b(0,1) = rs2b(0,1); Tl2b(0,2) = rs2b(0,2);
        Tl2b(1,0) = rs2b(1,0); Tl2b(1,1) = rs2b(1,1); Tl2b(1,2) = rs2b(1,2);
        Tl2b(2,0) = rs2b(2,0); Tl2b(2,1) = rs2b(2,1); Tl2b(2,2) = rs2b(2,2);
        Tl2b(0,3) = tfs2b.transform.translation.x;
        Tl2b(1,3) = tfs2b.transform.translation.y;
        Tl2b(2,3) = tfs2b.transform.translation.z;

        Tb2l(0,0) = rb2s(0,0); Tb2l(0,1) = rb2s(0,1); Tb2l(0,2) = rb2s(0,2);
        Tb2l(1,0) = rb2s(1,0); Tb2l(1,1) = rb2s(1,1); Tb2l(1,2) = rb2s(1,2);
        Tb2l(2,0) = rb2s(2,0); Tb2l(2,1) = rb2s(2,1); Tb2l(2,2) = rb2s(2,2);
        Tb2l(0,3) = vb2s(0);
        Tb2l(1,3) = vb2s(1);
        Tb2l(2,3) = vb2s(2);


        Eigen::Quaterniond qc2b;
        qc2b.x() = tfc2b.transform.rotation.x;
        qc2b.y() = tfc2b.transform.rotation.y;
        qc2b.z() = tfc2b.transform.rotation.z;
        qc2b.w() = tfc2b.transform.rotation.w;
        Eigen::Matrix3d rc2b = qc2b.normalized().toRotationMatrix();
        Eigen::Matrix3d rb2c = rc2b.inverse();
        Eigen::Vector3d vc2b, vb2c;
        vc2b(0) = tfc2b.transform.translation.x;
        vc2b(1) = tfc2b.transform.translation.y;
        vc2b(2) = tfc2b.transform.translation.z;
        vb2c = -rb2c*vc2b;

        Tc2b(0,0) = rc2b(0,0); Tc2b(0,1) = rc2b(0,1); Tc2b(0,2) = rc2b(0,2);
        Tc2b(1,0) = rc2b(1,0); Tc2b(1,1) = rc2b(1,1); Tc2b(1,2) = rc2b(1,2);
        Tc2b(2,0) = rc2b(2,0); Tc2b(2,1) = rc2b(2,1); Tc2b(2,2) = rc2b(2,2);
        Tc2b(0,3) = tfc2b.transform.translation.x;
        Tc2b(1,3) = tfc2b.transform.translation.y;
        Tc2b(2,3) = tfc2b.transform.translation.z;

        Tb2c(0,0) = rb2c(0,0); Tb2c(0,1) = rb2c(0,1); Tb2c(0,2) = rb2c(0,2);
        Tb2c(1,0) = rb2c(1,0); Tb2c(1,1) = rb2c(1,1); Tb2c(1,2) = rb2c(1,2);
        Tb2c(2,0) = rb2c(2,0); Tb2c(2,1) = rb2c(2,1); Tb2c(2,2) = rb2c(2,2);
        Tb2c(0,3) = vb2c(0);
        Tb2c(1,3) = vb2c(1);
        Tb2c(2,3) = vb2c(2);


        std::cout << Tl2b << std::endl;
        std::cout << Tc2b << std::endl;
        std::cout << Tb2l << std::endl;
        std::cout << Tb2c << std::endl;


        tf_received = true;
      }
      catch (tf2::TransformException &ex) {
        //ROS_WARN("%s",ex.what());
        continue;
      }
    }
    try{
      tfb2m = tfBuffer.lookupTransform("map", "base_link",
                                       ros::Time(0));

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
      //ROS_WARN("%s",ex.what());
      //continue;
    }
    seok::TimeChecker dur;
    // If data received
    if (!image_buf.empty() && !scan_buf.empty() && !bounding_buf.empty())
    {
      //ROS_INFO("buf no empty");
      lack_count = 0;

      double time;
      double time_r = image_buf.front()->header.stamp.toSec();
      double time_s = scan_buf.front()->header.stamp.toSec();
      double time_b = bounding_buf.front()->header.stamp.toSec();

      cv::Mat image;
      cv::Mat depth_image;

      sensor_msgs::LaserScan scan;
      door_angle::BoundingBoxes boxes;
      std_msgs::Header header;



      //rgb, scan, bounding box time sync 1.0 s
//      if (time_r < time_s - 0.05)
//      {
//        image_buf.pop();
//        ROS_INFO("pop rgb_image\n");
//      }
//      else if (time_r > time_s + 0.05)
//      {
//        scan_buf.pop();
//        ROS_INFO("rgb : %f", time_r);
//        ROS_INFO("s : %f", time_s);
//        ROS_INFO("pop scan\n");
//      }
//      else if(time_r < time_b - 0.05){
//        image_buf.pop();
//        ROS_INFO("pop rgb_image\n");
//      }
//      else if(time_r > time_b + 0.05){
//        bounding_buf.pop();
//        ROS_INFO("pop bound header\n");
//      }
//      else
        if(!bounding_buf.front()->bounding_boxes.empty())  //bounding box not empty && satisfy time sync
      {
        time = image_buf.front()->header.stamp.toSec();

        //convert sensor_msgs::image to cv::Mat
        cv_bridge::CvImageConstPtr ptr_l;
        ptr_l = cv_bridge::toCvCopy(image_buf.front(), sensor_msgs::image_encodings::BGR8);

        image = ptr_l->image.clone();
        header = image_buf.front()->header;
        image_buf.pop();

        // scan copy
        scan.ranges = scan_buf.front()->ranges;
        scan.header = scan_buf.front()->header;
        scan.angle_max = scan_buf.front()->angle_max;
        scan.angle_min = scan_buf.front()->angle_min;
        scan.range_max = scan_buf.front()->range_max;
        scan.range_min = scan_buf.front()->range_min;
        scan.scan_time = scan_buf.front()->scan_time;
        scan.intensities = scan_buf.front()->intensities;
        scan.time_increment = scan_buf.front()->time_increment;
        scan.angle_increment = scan_buf.front()->angle_increment;
        scan_buf.pop();
      }
      if(!image.empty() && !scan.ranges.empty()){
        //ROS_INFO("debug11111");
        boxes.bounding_boxes = bounding_buf.front()->bounding_boxes;
        std::cout << " number of bounding boxes : " << boxes.bounding_boxes.size() << std::endl;

        std::vector<Eigen::Vector4d> scanPoints;
        std::vector<Eigen::Vector3d> s2iPoints;
        double min_angle = static_cast<double>(scan.angle_min);
        double diff_angle = static_cast<double>(scan.angle_increment);


        // image lidar fusion
        for(unsigned long i = 0; i < scan.ranges.size(); i++){
          if(static_cast<double>(scan.ranges[i]) < 0.3) continue;
          double angle = min_angle + diff_angle * i;
          if(angle < -3.1415926535 / 2 || angle > 3.1415926535 / 2) continue;

          //ROS_INFO("angle : %f",angle);
          // get lidar point
          Eigen::Vector4d point_l;
          point_l(0) = std::cos(angle) * static_cast<double>(scan.ranges[i]);
          point_l(1) = std::sin(angle) * static_cast<double>(scan.ranges[i]);
          point_l(2) = 0.0; // z
          point_l(3) = 1.0;

          scanPoints.push_back(point_l);
          // frame convert
          // lidar => base_link frame => camera
          Eigen::Vector4d point_c;
          point_c = Tb2c * Tl2b * point_l;

          Eigen::Vector3d point_c3;
          point_c3(0) = point_c(0);
          point_c3(1) = point_c(1);
          point_c3(2) = point_c(2);

          Eigen::Vector3d point_uv;

          // camera => image plane
          point_uv = intrinsic * point_c3;

          point_uv(0) = point_uv(0) /  point_uv(2);
          point_uv(1) = point_uv(1) / point_uv(2);
          point_uv(2) = point_uv(2) / point_uv(2);

          // draw lidar point on image
          int r = static_cast<int>(point_uv(1));
          int c = static_cast<int>(point_uv(0));
          if(r > 1 && r < image.rows-1 && c > 1 && c < image.cols-1){
            for(int j = r-1; j < r+2; j++){
              for(int k = c - 1; k < c+2; k++){
                image.at<cv::Vec3b>(j,k)[0] = 255;
                image.at<cv::Vec3b>(j,k)[1] = 0;
                image.at<cv::Vec3b>(j,k)[2] = 0;
              }
            }
          }

          //scan to image point
          //s2iPoints.push_back(point_uv);
        }



        // bouding box fusion process
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        door_angle::DoorPoses doorPoses;
        pcl::PointXYZRGB point;

        // probability threshhold need!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        for(unsigned long i = 0; i < boxes.bounding_boxes.size(); i++){
          door_angle::BoundingBox box;
          box = boxes.bounding_boxes[0];
          if( box.Class == "handle"){
            ROS_INFO("handle");
            continue;
          }
          else if( box.Class == "door"){
            //pcl::PointCloud<pcl::PointXYZRGB> handleCloud;
            std::vector<Eigen::Vector2d> vecHandle;
            std::cout << "door!!!!" << std::endl;
            int xmin = static_cast<int>(box.xmin);
            int xmax = static_cast<int>(box.xmax);
            int ymin = static_cast<int>(box.ymin);
            int ymax = static_cast<int>(box.ymax);

            // bouding box visualize
            for(int c = xmin; c < xmax; c++){
              image.at<cv::Vec3b>(ymin,c)[0] = 0;
              image.at<cv::Vec3b>(ymin,c)[1] = 0;
              image.at<cv::Vec3b>(ymin,c)[2] = 255;

              image.at<cv::Vec3b>(ymax,c)[0] = 0;
              image.at<cv::Vec3b>(ymax,c)[1] = 0;
              image.at<cv::Vec3b>(ymax,c)[2] = 255;
            }
            for(int r = ymin + 1; r < ymax - 1; r++){
              image.at<cv::Vec3b>(r, xmin)[0] = 0;
              image.at<cv::Vec3b>(r, xmin)[1] = 0;
              image.at<cv::Vec3b>(r, xmin)[2] = 255;

              image.at<cv::Vec3b>(r, xmax)[0] = 0;
              image.at<cv::Vec3b>(r, xmax)[1] = 0;
              image.at<cv::Vec3b>(r, xmax)[2] = 255;
            }




            Eigen::Vector4d camera_origin = Eigen::Vector4d::Zero();
            camera_origin(3) = 1.0; // homogeneous

            Eigen::Vector4d camera_bounding_left, camera_bounding_right;
            camera_bounding_left(0) = (box.xmin - intrinsic(0,2)) / intrinsic(0,0);
            camera_bounding_left(1) = (box.ymin - intrinsic(1,2)) / intrinsic(1,1) ;
            camera_bounding_left(2) = 1.0;
            camera_bounding_left(3) = 1.0; // homogeneous

            camera_bounding_right(0) = (box.xmax - intrinsic(0,2)) / intrinsic(0,0);
            camera_bounding_right(1) = (box.ymin - intrinsic(1,2)) / intrinsic(1,1);
            camera_bounding_right(2) = 1.0;
            camera_bounding_right(3) = 1.0; // homogeneous

            // camera => base_link => lidar
            Eigen::Vector4d camera_origin_l = Tb2l * Tc2b * camera_origin;
            Eigen::Vector4d camera_bounding_left_l = Tb2l * Tc2b * camera_bounding_left;
            Eigen::Vector4d camera_bounding_right_l = Tb2l * Tc2b * camera_bounding_right;

            //Eigen::Vector2d vLeft, vRight;
            double x_diff_left = camera_bounding_left_l(0) - camera_origin_l(0);
            double y_diff_left = camera_bounding_left_l(1) - camera_origin_l(1);

            double x_diff_right = camera_bounding_right_l(0) - camera_origin_l(0);
            double y_diff_right = camera_bounding_right_l(1) - camera_origin_l(1);

            double a1 = y_diff_left / x_diff_left;
            double b1 = camera_bounding_left_l(1) - a1 * camera_bounding_left_l(0);
            // double c1  = - a1 * camera_origin_l(0) - b1 * camera_origin_l(1);
            //double c1  = - a1 * camera_bounding_left_l(0) - b1 * camera_bounding_left_l(1);

            double a2 = y_diff_right / x_diff_right;
            double b2 = camera_bounding_right_l(1) - a2 * camera_bounding_right_l(0);
            // double c2  = - a2 * camera_origin_l(0) - b2 * camera_origin_l(1);
            //double c2  = - a2 * camera_bounding_right_l(0) - b2 * camera_bounding_right_l(1);


            //std::cout << "a1 : " << a1 << " b1 : " << b1 << std::endl;
            //std::cout << "a2 : " << a2 << " b2 : " << b2 << std::endl;
            for(unsigned long i = 0; i < scanPoints.size(); i++){
              // scan point between plane contain lines from bounding box left Y, right Y, They are  perpendicular to the floor
              //              if( a1 < 0  && a2 < 0){
              if( (a1 * scanPoints[i](0) + b1) > scanPoints[i](1) && (a2 * scanPoints[i](0) + b2) < scanPoints[i](1) ){

                point.x = static_cast<float>(scanPoints[i](0));
                point.y = static_cast<float>(scanPoints[i](1));
                point.z = 0;
                point.b = 255;
                point.g = 0;
                point.r = 0;

                Eigen::Vector2d point2d;
                point2d(0) = static_cast<double>(point.x);
                point2d(1) = static_cast<double>(point.y);


                vecHandle.push_back(point2d);
                //handleCloud.push_back(point);
                cloud.push_back(point);
                //cloud.
              }

              // vector visualization DEBUG
              for(int i = -200; i < 200; i++){
                double x_deb = static_cast<double>(i) / 10.0;
                double y_deb1 = (a1 * x_deb + b1);
                double y_deb2 = (a2 * x_deb + b2);

                point.x = static_cast<float>(x_deb);
                point.y = static_cast<float>(y_deb1);
                point.z = 0;
                point.b = 255;
                point.g = 255;
                point.r = 0;

                cloud.push_back(point);

                point.x = static_cast<float>(x_deb);
                point.y = static_cast<float>(y_deb2);
                point.z = 0;
                point.b = 255;
                point.g = 0;
                point.r = 255;

                cloud.push_back(point);
              }
            }


            // calculate angle
            // RANSAC
            if(vecHandle.size() > 3){
              double threshHold = 0.1; // parameter!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
              int vecSize = static_cast<int>(vecHandle.size());

              int count = 60;
              srand(std::time(NULL));

              int maxInlierNum = 0;
              std::deque<int> dqMaxInlier;
              double maxA;
              double maxB;

              for(int i = 0; i < count; i++){
                unsigned long firstIndex = static_cast<unsigned long>(rand() % vecSize);
                unsigned long secondIndex = static_cast<unsigned long>(rand() % vecSize);

                if(firstIndex == secondIndex){
                  i--;
                  continue;
                }

                double a = ( vecHandle[firstIndex](1) - vecHandle[secondIndex](1) ) / ( vecHandle[firstIndex](0) - vecHandle[secondIndex](0) );
                double b = vecHandle[firstIndex](1) - a * vecHandle[firstIndex](0);
                double deno = std::sqrt(a*a + 1.0);

                std::deque<int> dqInlier;
                for(int j = 0; j < vecSize; j++){
                  if( ((a*vecHandle[j](0) -   vecHandle[j](1) + b ) / deno) <= threshHold){
                    dqInlier.push_front(j);
                  }
                }

                if(maxInlierNum < static_cast<int>(dqInlier.size())){
                  maxInlierNum = static_cast<int>(dqInlier.size());
                  dqMaxInlier = dqInlier;
                  maxA = a;
                  maxB = b;
                }
              }

              if(dqMaxInlier.size() > 1){
                door_angle::DoorPose doorPose;

                int index1 = dqMaxInlier.front();
                int index2 = dqMaxInlier.back();

                // convert coordinate !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                Eigen::Vector4d handle4d;

                handle4d(0) = vecHandle[static_cast<unsigned long>(index1)](0);
                handle4d(1) = vecHandle[static_cast<unsigned long>(index1)](1);
                handle4d(2) = 0.0;
                handle4d(3) = 1.0;

                handle4d = Tb2m * Tl2b * handle4d;

                doorPose.x1 = static_cast<float>(handle4d(0));
                doorPose.y1 = static_cast<float>(handle4d(1));

                handle4d(0) = vecHandle[static_cast<unsigned long>(index2)](0);
                handle4d(1) = vecHandle[static_cast<unsigned long>(index2)](1);
                handle4d(2) = 0.0;
                handle4d(3) = 1.0;

                handle4d = Tb2m * Tl2b * handle4d;

                doorPose.x2 = static_cast<float>(handle4d(0));
                doorPose.y2 = static_cast<float>(handle4d(1));

                std::cout << "a : " << maxA << ", b : " << maxB << std::endl;
                doorPoses.door_poses.push_back(doorPose);
              }
            }
          }
          else{
            continue;
          }
        }
        sensor_msgs::Image fusion_image;
        cv_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
        cv_bridge.toImageMsg(fusion_image);

        //publish fusion image
        fusion_image_pub.publish(fusion_image);

        //publish door point cloud
        pcl::toROSMsg(cloud, cloudmsg); //convert pcl::PointCloud<pcl::PointXYZRGB> to sensor_msgs::PointCloud2
        cloudmsg.header.frame_id = "base_footprint";
        cloud_door_pub.publish(cloudmsg);

        //publish door poses
        door_poses_pub.publish(doorPoses);
        dur.interval("loop while once");
      }
    }

    else{
      lack_count++;
      if(lack_count % 2000000 == 0){
        //ROS_INFO("wait messages");

        lack_count = 0;
      }
    }



    ros::spinOnce();
    //loop_rate.sleep();
  } // while

  return 0;
}




// keep code

// ///// code for draw lidar z = 0 plane (x-y plane) on image => wrong
//        Eigen::Vector4d xy_plane, origin_l;
//        xy_plane(0) = 0; xy_plane(1) = 0; xy_plane(2) = 1; xy_plane(3) = 0; // normal vector of xy plane
//        origin_l(0) = 0; origin_l(1) = 0; origin_l(2) = 0; origin_l(3) = 1; // origin of lidar

//        Eigen::Vector4d xy_plane_c = Tb2c * Tl2b * xy_plane; // lidar => base_link => camera
//        Eigen::Vector4d origin_lc = Tb2c * Tl2b * origin_l; // lidar => base_link => camera
//        Eigen::Vector3d xy_plane_c3, origin_lc3;

//        xy_plane_c3(0) = xy_plane_c(0);
//        xy_plane_c3(1) = xy_plane_c(1);
//        xy_plane_c3(2) = xy_plane_c(2);

//        origin_lc3(0) = origin_lc(0);
//        origin_lc3(1) = origin_lc(1);
//        origin_lc3(2) = origin_lc(2);

//        // xy_plane normal vector in uv space
//        Eigen::Vector3d xy_plane_uv = intrinsic * xy_plane_c3; // camera => image
//        //Eigen::Vector3d origin_l_uv = intrinsic * origin_lc3; // camera => image


//        double mag = xy_plane_uv(0) * xy_plane_uv(0) + xy_plane_uv(1) * xy_plane_uv(1) + xy_plane_uv(2) * xy_plane_uv(2);
//        mag = std::sqrt(mag);

//        std::cout << " 0 : " << xy_plane_uv(0) << " 1 : " << xy_plane_uv(1) << " 2 : " << xy_plane_uv(2) << std::endl;

//        xy_plane_uv(0) = xy_plane_uv(0) / mag;
//        xy_plane_uv(1) = xy_plane_uv(1) / mag;
//        xy_plane_uv(2) = xy_plane_uv(2) / mag;

//        //origin_l_uv(0) = origin_l_uv(0) / origin_l_uv(2);
//        //origin_l_uv(1) = origin_l_uv(1) / origin_l_uv(2);
//        //origin_l_uv(2) = origin_l_uv(2) / origin_l_uv(2);

//        std::cout << " 0' : " << xy_plane_uv(0) << " 1' : " << xy_plane_uv(1) << " 2' : " << xy_plane_uv(2) << std::endl;


//        double a = xy_plane_uv(0);
//        double b = xy_plane_uv(1);
//        double c = xy_plane_uv(2);

//        // lidar origin point on image_plane
//        //int r_lo = static_cast<int>(origin_l_uv(0));
//        //int c_lo = static_cast<int>(origin_l_uv(1));
//        std::cout << " a : " << a << " b : " << b <<  " c : " << c << std::endl;

//        if(b != 0.0){
//          for(int i = 0; i < image.cols; i++){
//            // center coordinate, intersection xy-plane with image
//            int u = i - static_cast<int>(intrinsic(0,2));
//            int v = static_cast<int>(-u * a/b - c/b);

//            // left-up coordinate
//            int r = v + static_cast<int>(intrinsic(1,2));
//            int c = u + static_cast<int>(intrinsic(0,2));

//            //std::cout << " u : " << u << " v : " << v << std::endl;
//            if(r > 0 && r < image.rows && c > 0 && c < image.cols){

//              image.at<cv::Vec3b>(r,c)[0] = 255;
//              image.at<cv::Vec3b>(r,c)[1] = 0;
//              image.at<cv::Vec3b>(r,c)[2] = 0;
//            }
//          }
//        }
//        else{
//          for(int i = 0; i < image.rows; i++){
//            // center coordinate, intersection xy-plane with image
//            int v = i - static_cast<int>(intrinsic(1,2));
//            int u = static_cast<int>(-c/a);

//            // left-up coordinate
//            int r = v + static_cast<int>(intrinsic(1,2));
//            int c = u + static_cast<int>(intrinsic(0,2));

//            std::cout << " u : " << u << " v : " << v << std::endl;
//            if(v > 0 && v < image.rows && u > 0 && u < image.cols){

//              image.at<cv::Vec3b>(v,u)[0] = 255;
//              image.at<cv::Vec3b>(v,u)[1] = 0;
//              image.at<cv::Vec3b>(v,u)[2] = 0;
//            }

//          }

//        }
