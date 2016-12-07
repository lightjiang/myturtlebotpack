"""
light
20161201

rosrun rqt_graph rqt_graph
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/turtlebot/catkin_ws/src/myturtlebotpack/map/floor2.yaml

roslaunch myturtlebotpack my.launch
roslaunch myturtlebotpack amcl_demo.launch

roslaunch turtlebot_rviz_launchers view_navigation.launch --screen
roslaunch turtlebot_teleop keyboard_teleop.launch

/home/workstation/catkin_ws/src/myturtlebotpack/laser_sdk/output/Linux/Release/ultra_simple
/home/workstation/catkin_ws/src/myturtlebotpack/laser_sdk/output/Linux/Release/simple_grabber  /dev/ttyUSB0
"""
