#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
using namespace std;

float est_theta, est_x, est_y, ref_theta, ref_x, ref_y;
ros::Publisher robot_vel;
float L_vel = 0.15, radius = 0.08;
double T0,TF;
void estimacion(const geometry_msgs::Pose2D::ConstPtr& msg);
void referencia(const geometry_msgs::Pose2D::ConstPtr& msg);
void control();
int main(int argc, char** argv)
{

	ros::init(argc, argv, "control_cmd_vel");
  ros::NodeHandle n;
  robot_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  ros::Subscriber est = n.subscribe("/pose_TG", 100, estimacion);
  ros::Subscriber des = n.subscribe("/pose_ref", 100, referencia);
	ros::Rate rate(10);
  T0=ros::Time::now().toSec();
  while (n.ok())
  {
    control();
    
		T0=TF;
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}

void estimacion(const geometry_msgs::Pose2D::ConstPtr& msg)
{ 
  est_x = msg->x;
  est_y = msg->y;
  est_theta = msg->theta;
}
void referencia(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  ref_x = msg->x;
  ref_y = msg->y;
  ref_theta = msg->theta;
}
void control(){

  geometry_msgs::Twist vel_msg;
  float e_x = ref_x-est_x;
  float e_y = ref_y-est_y;
  float e_theta = ref_theta-est_theta;
  if (stop)
  {
    vel_msg.angular.z = 1.2*e_theta + e_y;
    if (vel_msg.angular.z > L_vel/radius)
				vel_msg.angular.z = L_vel/radius;
			if (vel_msg.angular.z < -L_vel/radius)
				vel_msg.angular.z = -L_vel/radius;

    if (abs(e_theta) < M_PI/18.0)
      enable_v = true;
    if (enable_v)
    {
      vel_msg.linear.x = e_x*(1-abs(e_theta)/M_PI_2);
      if (vel_msg.linear.x> L_vel)
        vel_msg.linear.x=L_vel;
      if (vel_msg.linear.x<0)
        vel_msg.linear.x=0;
    }else
      vel_msg.linear.x = 0;
    
 
  }else
  {
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
  }
  robot_vel.publish(vel_msg);
}