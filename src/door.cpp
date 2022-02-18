#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include <opencv2/opencv.hpp>

nav_msgs::OccupancyGrid map;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_ptr)
{
  map.data = map_ptr->data;
  map.info = map_ptr->info;
  map.header = map_ptr->header;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "door");
  ros::NodeHandle nh;
  ros::Subscriber subscribe_map = nh.subscribe("map", 1000, mapCallback);
  ros::Publisher  publish_map   = nh.advertise<nav_msgs::OccupancyGrid>("map_door",1000);

  ros::Rate loop_rate(50);

  nav_msgs::OccupancyGrid map_door;
  bool first = true;
  int width, height;
  while (ros::ok())
  {
    if(first && !map.data.empty()){
      map_door.data   = map.data;
      map_door.info   = map.info;
      map_door.header = map.header;
      width           = map.info.width;
      height          = map.info.height;
      first = false;
    }
    if(!map_door.data.empty()){
      //real world coordinate
      double x1 = -0.704240620136;
      double y1 = 3.02599477768;
      double x2 = 0.895031690598;
      double y2 = 3.71079492569;

      double tmp_doorAngle = std::atan2(y2-y1,x2-x1);
      std::cout << tmp_doorAngle << std::endl;

      float resolution = map_door.info.resolution;
      double ratio = 1.0 / static_cast<double>(resolution);

      //map coordinate
      double c1 = std::min(x1,x2) * ratio;
      double r1 = std::min(y1,y2) * ratio;
      double c2 = std::max(x1,x2) * ratio;
      double r2 = std::max(y1,y2) * ratio;

      //bounding box



    }







    publish_map.publish(map_door);
    ros::spinOnce();
  }

  return 0;
}
