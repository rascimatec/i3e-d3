#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::Twist odom_twist;

void twistOdomCallback(const geometry_msgs::TwistConstPtr& vel) {
  odom_twist = *vel;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle nh;
  
  ros::Publisher odom_pub;
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
  ros::Subscriber twist_odom_sub;
    twist_odom_sub = nh.subscribe<geometry_msgs::Twist>("twist_odom", 1, &twistOdomCallback);
    
  tf2_ros::TransformBroadcaster odom_broadcaster;

  double x = 0.0, y = 0.0, th = 0.0;
  double delta_x, delta_y, delta_th;
  double dt;

  ros::Time current_time, last_time;
  
  last_time = ros::Time::now();
  ros::Rate rate(10);

  while (nh.ok()) {
    current_time = ros::Time::now();

    // Compute odometry given the velocities of the robot
    dt = (current_time - last_time).toSec();
    delta_x = odom_twist.linear.x * cos(th) * dt;
    delta_y = odom_twist.linear.x * sin(th) * dt;
    delta_th = odom_twist.angular.z * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    // Publish the transform over TF
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, th);
    geometry_msgs::Quaternion odom_quat = tf2::toMsg(quat_tf);

    geometry_msgs::TransformStamped odom_transform;
    odom_transform.header.stamp = current_time;
    odom_transform.header.frame_id = "odom";
    odom_transform.child_frame_id = "base_link";

    odom_transform.transform.translation.x = x;
    odom_transform.transform.translation.y = y;
    odom_transform.transform.translation.z = 0.0;
    odom_transform.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_transform);

    // Publish the odometry message over ROS
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.child_frame_id = "base_link";
    odom_msg.twist.twist = odom_twist;

    odom_pub.publish(odom_msg);

    last_time = current_time;
    rate.sleep();
    ros::spinOnce();
  }
}
