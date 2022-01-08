#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "vel_publisher");
  ros::NodeHandle nh;

  geometry_msgs::Twist vel_msg;
  
  ros::Publisher vel_pub;
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  ros::Rate rate(10);
  while (nh.ok()) {
    vel_msg.linear.x = 0.5;
    vel_pub.publish(vel_msg);
    rate.sleep();
  }

}
