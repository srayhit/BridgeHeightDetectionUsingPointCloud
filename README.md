# BridgeHeightDetectionUsingPointCloud

It is not rare to find situations where a truck/RV or other tall vehicle is unable to
traverse a certain road due to the presence of a low-height bridge on the route.
We propose a system to detect the height of a bridge beforehand using monocular
depth perception, to determine whether the particular vehicle with known height
will be able to pass through a particular bridge or tunnel. If a vehicle does not
meet the clearance for a particular bridge, our ADAS system will be able to warn
the driver and suggest or force a stop, thus avoiding a dangerous collision. Our
system is able to process multiple depth clouds per second and extract distances
between dissimilar parallel planes. The maximum error recorded over multiple
bridge heights was less than 3 cm.

# Block Diagram
![alt text](https://github.com/srayhit/BridgeHeightDetectionUsingPointCloud/blob/master/blockDiagram.png "Block Diagram")
# Requirements

1. ROS Indigo or Kinetic
2. Gazebo
3. Turtlebot 2
4. ASUS RGB-D camera with openNI2 camera driver
5. A PC/laptop with ubuntu 16.04
6. A local Wifi hotspot to communicate with the turtlebot and PC

# Point Cloud View from the Gazebo simulaltion

![alt text](https://github.com/srayhit/BridgeHeightDetectionUsingPointCloud/blob/master/pointCloudViewGazebo.png "Block Diagram")
# Topics covered

1. ROS
2. RANSAC algorithm for detecting parallel planes from point cloud data
3. Record point cloud data from .bag files using RGB-D camera
4. Turtlebot
5. Gazebo
6. Python and C++
