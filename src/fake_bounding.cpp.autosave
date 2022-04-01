#include "ros/ros.h"
#include "std_msgs/String.h"

#include "door_angle/BoundingBox.h"
#include "door_angle/BoundingBoxes.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_bounding");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<door_angle::BoundingBoxes>("/bounding_box_array", 1000);

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    door_angle::BoundingBoxes msg;
    door_angle::BoundingBox box;
    box.xmin = 100;
    box.xmax = 500;
    box.ymin = 100;
    box.ymax = 400;
    box.Class = "handles";
    box.probability = 1.0;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "camera";
    msg.bounding_boxes.push_back(box);
    msg.bounding_boxes.push_back(box);

    chatter_pub.publish(msg);
    ROS_INFO("pub fake bounding");

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
