#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TeleopD3{
public:
    TeleopD3();
    
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    int linear_, angular_;
    double l_scale_, a_scale_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;

};

TeleopD3::TeleopD3(): linear_(1), angular_(0){

    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    nh_.param("scale_angular", a_scale_, a_scale_);

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/i3e_d3_velocity_controller/cmd_vel", 1);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopD3::joyCallback, this);

}

void TeleopD3::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
    
    geometry_msgs::Twist twist;
    twist.linear.x = l_scale_*joy->axes[linear_];
    twist.angular.z = a_scale_*joy->axes[angular_];
    vel_pub_.publish(twist);
}


int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "/teleop_i3e_d3");
    TeleopD3 teleop_d3;

    ros::spin();
}
