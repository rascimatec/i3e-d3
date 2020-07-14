#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "/vel_publisher");
    ros::NodeHandle nh;

    ros::Publisher vel_publihsher;
    vel_publihsher = nh.advertise<geometry_msgs::Twist>("/i3e_d3_velocity_controller/cmd_vel", 10);
    
    geometry_msgs::Twist vel;
    vel.linear.x = 0.7;
    vel.angular.z = 0.5;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        vel_publihsher.publish(vel);
    
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
