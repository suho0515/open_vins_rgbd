#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

nav_msgs::Path path;

ros::Publisher path_pub;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  path.header = msg->header;
  path.poses.push_back(*msg);

  path_pub.publish(path);  
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "path_publisher");


  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("/vrpn_client_node/handheld/pose", 1000, poseCallback);

  path_pub = n.advertise<nav_msgs::Path>("path", 1000);

  ros::spin();

  return 0;
}
