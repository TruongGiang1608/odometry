#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

double v_x, v_y, w;

void velCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    v_x = msg->linear.x;
    v_y = msg->linear.y;
    w = msg->angular.z;
  }
int main(int argc, char **argv)
{
ros::init(argc,argv,"inv_kinematic");
ros::NodeHanle n;
ros::Publisher v_wheels_pub = a.advertise<std_msgs::Float64MultiArray>("joint_velocity/command",1000);

ros::Subscriber v_robot_sub = n.subscribe("/cmd_vel",1000,velCallback);

const double PI = acos(-1.0);
const double R = 0.0625;
const double L = 0.28;

ros::Rate rate(50);
while(n.ok())
  {
  ros::spinOnce();
  ros::Time current_time;
  current_time = ros::Time::now();
  // Tinh van toc dai 3 banh theo van co robot trong he toa do gan voi robot
  double v1 = w*L + v_y;
  double v2 = w*L - v_x*sin(PI/3) - v_y*cos(PI/3);
  double v3 = w*L + v_x*sin(PI/3) - v_y*cos(PI/3);
  
  std_msgs::Float64MultiArray v_wheels_cmd;
  v_wheels_cmd.data[0] = v1;
  v_wheels_cmd.data[1] = v2;
  v_wheels_cmd.data[2] = v3;
  
  v_wheels_pub.publish(v_wheels_cmd);
  
  rate.sleep();
  }
}
