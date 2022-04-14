#include <queue>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "sensor_msgs/Image.h"

#include "door_angle/BoundingBox.h"
#include "door_angle/BoundingBoxes.h"

std::queue<sensor_msgs::ImageConstPtr>  image_buf;

void imgCallback(const sensor_msgs::ImageConstPtr &image)
{
  image_buf.push(image);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_bounding");
  ros::NodeHandle nh;

  std::string image_topic;
  nh.param<std::string>("image_topic",image_topic,"/camera/color/image_raw");

  ros::Publisher chatter_pub = nh.advertise<door_angle::BoundingBoxes>("/bounding_box_array", 1000);
  ros::Subscriber image_sub = nh.subscribe(image_topic, 1000, imgCallback);

  ros::Rate loop_rate(60);
  while (ros::ok())
  {
    if(!image_buf.empty()){
      door_angle::BoundingBoxes msg;
      door_angle::BoundingBox box;
      box.xmin = 100;
      box.xmax = 500;
      box.ymin = 100;
      box.ymax = 400;
      box.Class = "door";
      box.probability = 1.0;
      msg.header.stamp = image_buf.front()->header.stamp;
      image_buf.pop();
      msg.header.frame_id = "camera";
      msg.bounding_boxes.push_back(box);
      msg.bounding_boxes.push_back(box);

      chatter_pub.publish(msg);
      ROS_INFO("pub fake bounding");
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
