#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include "cmath"
#define  THRESHOLD_DISTANCE 0.05
#define  KX 1.0
#define  KY 1.50
#define  L_VEL_MAX 0.02
#define  W_VEL_MAX 0.5
using namespace std;
bool flag = 0;
geometry_msgs::Pose2D p_est, p_ref;
ros::Publisher robot_vel;
void estimacion(const geometry_msgs::Pose2D::ConstPtr& msg);
void referencia(const geometry_msgs::Pose2D::ConstPtr& msg);
void control();

 void controlCallback(const ros::TimerEvent&);

int main(int argc, char** argv)
{

  ros::init(argc, argv, "control_cmd_vel");
  ros::NodeHandle n;
  robot_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  ros::Subscriber est = n.subscribe("/Pose_TG", 100, estimacion);
  ros::Subscriber des = n.subscribe("/Pose_ref", 100, referencia);
  ros::Timer timer =  n.createTimer(ros::Duration(1.0/10.0),controlCallback);
  ros::spin();
  return 0;
}

void estimacion(const geometry_msgs::Pose2D::ConstPtr& msg)
{ 
  p_est.x = msg->x;
  p_est.y = msg->y;
}
void referencia(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  flag = 1;
  p_ref.x = msg->x;
  p_ref.y = msg->y;
}
void controlCallback(const ros::TimerEvent&){

  geometry_msgs::Twist vel_msg;
  geometry_msgs::Pose2D e;
  e.x = p_ref.x - p_est.x;
  e.y = p_ref.y - p_est.y;
  vel_msg.angular.z = 0;
  vel_msg.linear.x = 0;
   if ( sqrt(pow(e.x,2.0) + pow(e.y,2.0)) > THRESHOLD_DISTANCE && flag == 1)
   {
	vel_msg.angular.z = KY*e.y;
	
     if (vel_msg.angular.z > W_VEL_MAX)
	vel_msg.angular.z = W_VEL_MAX;
     if (vel_msg.angular.z < -W_VEL_MAX)
        vel_msg.angular.z = -W_VEL_MAX;
	
       vel_msg.linear.x = KX*e.x;
	
      if (vel_msg.linear.x> L_VEL_MAX)
         vel_msg.linear.x=L_VEL_MAX;
      if (vel_msg.linear.x<0)
         vel_msg.linear.x=0;      
  }
  robot_vel.publish(vel_msg);
  ros::spinOnce();
}
