# door_angle
#### This package aims to get door pose using RGB-D data

fake_bounding : Publish fake bounding box for debugging.  
plane_seg     : Make point cloud using bounding box & RGB-D. And segment plane using point cloud.  
storage_      : Publish MarkerArray for mark the door position. And manage document that contains door positions.  
camera_lidar_door : draw lidar point on image, calculate bounding box's real worl coordinate.  
door_register : It manage the data of door pose. And also visualize it  
disinfection_module : Read door pose data, calculate robot pose, make path & publish goal for disinfection.  
