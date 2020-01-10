#include "ros/ros.h"
#include "nav_msgs/Odometry.h"  //nav_msgs is the package (but odom is the topic)

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  ROS_INFO("x: %f. y: %f", ???, ???);
}


int main(int argc, char** argv){

  ros::init(argc, argv, "location_monitor");
  ros::NodeHandle nh;
  ros::Subscriber = nh.subscribe("odom", 10, OdomCallback);
  ros::spin();
  return 0;

}
