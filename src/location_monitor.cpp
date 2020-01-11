// #include "ros/ros.h"
// #include "nav_msgs/Odometry.h"  //nav_msgs is the package (but odom is the topic)
//
// This is the Callback that will contain the data from the /odom topic
// void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
//   double x = msg->pose.pose.position.x; // rosmsg show nav_msgs/Odometry
//   double y = msg->pose.pose.position.y;
//   ROS_INFO("x: %f. y: %f", x, y); // its a way of logging in ROS (specifically in cpp)
// }
//
//
// int main(int argc, char** argv){
//
//   ros::init(argc, argv, "location_monitor");
//   ros::NodeHandle nh;
//   ros::Subscriber sub = nh.subscribe("odom", 10, OdomCallback);
//   ros::spin();
//   return 0;
//
// }

#include <vector>
#include <string>
#include <math.h>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"  //nav_msgs is the package (but odom is the topic)
#include "robot_location_monitor/LandmarkDistance.h"

using std::vector;
using std::string;
using robot_location_monitor::LandmarkDistance;
// This is basically a struct that tells you the name of the landmark and the position(x, y)
class Landmark{
 public:
  Landmark(string name, double x, double y)
    : name(name), x(x), y(y) {}
  string name;
  double x;
  double y;
};


class LandmarkMonitor{
 public:
   LandmarkMonitor(const ros::Publisher& landmark_pub): landmarks_(), landmark_pub_(landmark_pub){
     InitLandmarks();
   }

  // This is the Callback that will contain the data from the /odom topic
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
   double x = msg->pose.pose.position.x; // rosmsg show nav_msgs/Odometry
   double y = msg->pose.pose.position.y;
   LandmarkDistance ld = FindClosest(x, y);
   // ROS_INFO("x: %f. y: %f", x, y); // its a way of logging in ROS (specifically in cpp)
   // ROS_INFO("name: %s. d: %f", ld.name.c_str(), ld.distance);
   landmark_pub_.publish(ld);
  }

  private:
    vector<Landmark> landmarks_;
    ros::Publisher landmark_pub_;

    LandmarkDistance FindClosest(double x, double y){
      LandmarkDistance result;
      result.distance = -1;

      for (size_t i = 0; i < landmarks_.size(); i++){
        const Landmark&  landmark = landmarks_[i];
        double xd = landmark.x - x;
        double yd = landmark.y - y;
        double distance = sqrt(xd *xd + yd*yd);

        if (result.distance < 0 || distance < result.distance){
          result.name = landmark.name;
          result.distance = distance;
        }
      }
      return result;
    }

    void InitLandmarks(){
      landmarks_.push_back(Landmark("main_entrance", 2.25, -0.03));
      landmarks_.push_back(Landmark("side_entrance_1", 1.45, 1.96));
      landmarks_.push_back(Landmark("side_entrance_2", 1.35,-1.97));
      landmarks_.push_back(Landmark("side_entrance_3", -1.45, 1.96));
      landmarks_.push_back(Landmark("side_entrance_4", -1.47, -1.96));
      landmarks_.push_back(Landmark("center", -0.17, 0.06));
    }
};



int main(int argc, char** argv){
  ros::init(argc, argv, "location_monitor");
  ros::NodeHandle nh;
  ros::Publisher landmark_pub = nh.advertise<LandmarkDistance>("closest_landmark", 10);
  LandmarkMonitor monitor(landmark_pub);
  ros::Subscriber sub = nh.subscribe("odom", 10, &LandmarkMonitor::OdomCallback, &monitor);
  ros::spin();
  return 0;

}
