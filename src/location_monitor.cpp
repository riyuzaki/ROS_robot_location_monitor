#include "ros/ros.h"
#include "nav_msgs/Odometry.h"  //nav_msgs is the package (but odom is the topic)

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  double x = msg->pose.pose.position.x; // rosmsg show nav_msgs/Odometry
  double y = msg->pose.pose.position.y;
  ROS_INFO("x: %f. y: %f", x, y); // its a way of logging in ROS (specifically in cpp)
}


int main(int argc, char** argv){

  ros::init(argc, argv, "location_monitor");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("odom", 10, OdomCallback);
  ros::spin();
  return 0;

}
