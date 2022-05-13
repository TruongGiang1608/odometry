#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <sstream>

double V1_rd, V2_rd, V3_rd;
void velCallback1(const std_msgs/Float64MultiArray& msg1)
{
  V1_rd = msg1.data[0];
  V2_rd = msg1.data[1];
  V3_rd = msg1.data[2];
}
double V1_ref, V2_ref, V3_ref;
void velCallback2(const std_msgs/Float64MultiArray& msg2)
{
  V1_ref = msg2.data[0];
  V2_ref = msg2.data[1];
  V3_ref = msg2.data[2];
}
int main(int argc, char **argv)
{
  ros::init(argc,argv,"odometry_publisher");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",1000);
  tf::TransformBroadcaster odom_broadcaster;
  ros::Subscriber vel_sub = n.subscribe("wheels_speed",1000,velCallback1);
  ros::Subscriber vel_sub_ref = n.subscribe("joint_velocity/command",1000,velCallback2);

  const double PI = acos(-1.0);
  const double R = 0.0625;
  const double L = 0.28;

  double x = 0.0;
  double y = 0.0;
  double phi = 0.0;
  
  double V1_bf = V1_ref;
  double V2_bf = V2_ref;
  double V3_bf = V3_ref;
  
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  ROS_INFO("Publishing odometry information over ROS");
  ros::Rate rate(50);
while(n.ok())
{
  ros::spinOnce();
  if fabs(V1_rd - V1_ref) >= 0.1*V1_rd
    {
    V1 = V1_bf;
    }
  else {V1 = V1_rd;}
  if fabs(V2_rd - V2_ref) >= 0.1*V2_rd
    {
    V2 = V2_bf;
    }
  else {V2 = V2_rd;}
  if fabs(V3_rd - V3_ref) >= 0.1*V3_rd
    {
    V3 = V3_bf;
    }
  else {V3 = V3_rd;}
  
  V1_bf = V1;
  V2_bf = V2;
  V3_bf = V3;
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
  odom_trans.child_frame_id = "base_footprint"
  
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
