// Teleop configuration for a generic joystick
// 'L3' up/down for linear velocity, left/right for angular velocity
// 'L1' or 'R1' to stop the robot
//  Hold '3' to enable turbo
//  Hold '1' to move slowly
//
//           L2                                       R2
//          L1                                       R1
//       _=====_                                  _=====_
//      / _____ \                                / _____ \
//    +.-'_____'-.------------------------------.-'_____'-.+
//   /   |     |  '.        X X X X           .'  |     |   \
//  / ___| /|\ |___ \                        / ___|  1  |___ \      
// / |      |      | ;    |9|       |10 >   ; |             ||
// | | <---   ---> | |                      | | 4         2 ||       
// | |___   |   ___| ;                      ; |___       ___||
// |\    | \|/ |    /  _      ____      _   \    |  3  |    /|      
// | \   |_____|  .','" "',  (MODE)  ,'" "', '.  |_____|  .' |
// |  '-.______.-' /       \        /       \  '-._____.-'   |
// |               |  L3   |--------|  R3   |                |
// |              /\       /        \       /\               |
// |             /  '.___.'          '.___.'  \              |
// |            /                              \             |
//  \          /                                \           /
//   \________/                                  \_________/
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TeleopD3{
public:
    TeleopD3();
    void moveD3();
    
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    geometry_msgs::Twist twist_;
    ros::NodeHandle nh_;

    int reverse_;
    bool rev_;
    int stop1_, stop2_;
    int turbo_, slow_;
    int linear_, angular_;
    double l_scale_, a_scale_, k_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;

};

TeleopD3::TeleopD3(): linear_(1), angular_(0){

    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("stop_button1", stop1_, stop1_);
    nh_.param("stop_button2", stop2_, stop2_);
    nh_.param("turbo_button", turbo_, turbo_);
    nh_.param("slow_button", slow_, slow_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("reverse_button", reverse_, reverse_);

    rev_ = true;

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("velocity_controller/cmd_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopD3::joyCallback, this);

}

void TeleopD3::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){

    if(joy->buttons[reverse_]) rev_ = !rev_;
    
    k_ = 0.4;
    if(joy->buttons[turbo_]) k_ = 1;
    if(joy->buttons[slow_]) k_ = 0.2;
    if(joy->buttons[stop1_] || joy->buttons[stop2_]) k_ = 0;
    
    twist_.linear.x = k_*l_scale_*joy->axes[linear_];
    
    if(joy->axes[linear_] < 0 && rev_) k_ = -k_;
    twist_.angular.z = k_*a_scale_*joy->axes[angular_];

}

void TeleopD3::moveD3(){    
    vel_pub_.publish(twist_); 
}


int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "/teleop_i3e_d3");
    TeleopD3 teleop_d3;

    ros::Rate loop_rate(10);
    while (ros::ok()){
        teleop_d3.moveD3();

        ros::spinOnce();
        loop_rate.sleep();
    }
}
