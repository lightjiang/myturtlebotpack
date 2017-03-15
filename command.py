"""
light
20161201

git clone https://github.com/lightjiang/myturtlebotpack.git

rosrun rqt_graph rqt_graph
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/turtlebot/catkin_ws/src/myturtlebotpack/map/test21.yaml
roslaunch turtlebot_navigation gmapping_demo.launch
`

roslaunch myturtlebotpack my.launch
roslaunch myturtlebotpack amcl_demo.launch

roslaunch turtlebot_rviz_launchers view_navigation.launch --screen
roslaunch turtlebot_teleop keyboard_teleop.launch

roslaunch kobuki_auto_docking minimal.launch

/home/workstation/catkin_ws/src/myturtlebotpack/laser_sdk/output/Linux/Release/ultra_simple
/home/workstation/catkin_ws/src/myturtlebotpack/laser_sdk/output/Linux/Release/simple_grabber  /dev/ttyUSB0

roslaunch myturtlebotpack gmapping_demo.launch

rosrun rqt_graph rqt_graph

catkin_make -DCATKIN_WHITELIST_PACKAGES=""

"""
