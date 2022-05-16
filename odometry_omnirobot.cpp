#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>
#include <sstream>

double V1, V2, V3;
double V1_rd, V2_rd, V3_rd;
void velCallback(const std_msgs::Float64MultiArray& msg)
{
  V1_rd = msg.data[1];
  V2_rd = msg.data[0];
  V3_rd = msg.data[2];
}
double V1_ref, V2_ref, V3_ref;
void velCallback1(const std_msgs::Float64& msg1)
{
  V1_ref = msg1.data;
}
void velCallback2(const std_msgs::Float64& msg2)
{
  V2_ref = msg2.data;
}
void velCallback3(const std_msgs::Float64& msg3)
{
  V3_ref = msg3.data;
}
int main(int argc, char **argv)
{
  ros::init(argc,argv,"odometry_publisher");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",1000);
  tf::TransformBroadcaster odom_broadcaster;
  ros::Subscriber vel_sub = n.subscribe("wheels_speed",1000,velCallback);
  
  ros::Subscriber vel1_sub_ref = n.subscribe("/robot_kist/joint_1_velocity/command",1000,velCallback1);
  ros::Subscriber vel2_sub_ref = n.subscribe("/robot_kist/joint_2_velocity/command",1000,velCallback2);
  ros::Subscriber vel3_sub_ref = n.subscribe("/robot_kist/joint_3_velocity/command",1000,velCallback3);

  const double PI = acos(-1.0);
  const double R = 0.0625;
  const double L = 0.28;

  double x = 0.0;
  double y = 0.0;
  double phi = 0.0;
  
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  ROS_INFO("Publishing odometry information over ROS");
  ros::Rate rate(50);
while(n.ok())
{
  ros::spinOnce();
  if (fabs(V1_rd - V1_ref) <= 0.001)
    {
    V1 = V1_ref;
    }
  else {V1 = V1_rd;}
  if (fabs(V2_rd - V2_ref) <= 0.001)
    {
    V2 = V2_ref;
    }
  else {V2 = V2_rd;}
  if (fabs(V3_rd - V3_ref) <= 0.001)
    {
    V3 = V3_ref;
    }
  else {V3 = V3_rd;}
  
  current_time = ros::Time::now();
  
  double dt = (current_time - last_time).toSec();
  double V_x = -(2/3)*V1*sin(phi) + (sin(phi)-sqrt(3)*cos(phi))*V2/3 + (sin(phi)+sqrt(3)*cos(phi))*V3/3;
  double V_y = (2/3)*V1*cos(phi) - (sqrt(3)*sin(phi)+cos(phi))*V2/3 + (sqrt(3)*sin(phi)-cos(phi))*V3/3;
  double w = (V1 + V2 + V3)/(3*L);
  
    
  double x = x + V_x*dt;
  double y = y + V_y*dt;
  double phi = phi + w*dt;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(phi);
  
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_footprint";
  
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  
  odom_broadcaster.sendTransform(odom_trans);
  
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  
  odom.child_frame_id = "base_footprint";
  odom.twist.twist.linear.x = V_x;
  odom.twist.twist.linear.y = V_y;
  odom.twist.twist.angular.z = w;
  
  odom_pub.publish(odom);
  last_time = current_time;
  
  rate.sleep();
  }
}
