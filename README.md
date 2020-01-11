# ROS_robot_location_monitor
 This is a simple ROS package to monitor at which landmark the robot is near to.
 I have used turtlebot robot simulation in gazebo for this purpose.

 Clone this repo within your catkin workspace and build it using `catkin build` and
 run the location monitor node using `rosrun robot_location_monitor location_monitor_node`.
 This will create a new topic `closest_landmark`.You can move the robot in gazebo and
 observe the published landmark and distance results in `rostopic echo closest_landmark`
 or subscribe to `closest_landmark` and get the results.
