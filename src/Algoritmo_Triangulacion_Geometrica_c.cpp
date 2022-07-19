#include <ros/ros.h>
#include "curso_pdi/aruco.h"
#include "curso_pdi/VectorAruco.h"
#include <opencv2/highgui/highgui.hpp>
#include "cmath"
#include <geometry_msgs/Pose2D.h>
#include <iostream>
#include <fstream>
#define pi 3.141592653589793
using namespace std;
using namespace cv;
using namespace curso_pdi;
using namespace geometry_msgs;
/*-------------   Variables -----------------*/
float lambda_12 = 0, lambda_31 = 0,
      phi = 0, sigma = 0, gamma_tg = 0, long_12 = 0, long_31 = 0,
      tau = 0,long_1 = 0, X_r = 0, Y_r = 0, Theta_r = 0;
double lambda[3] = {};
float d = 0.5*8;
Point2f landmark[] = {Point2f( 0,  -1),Point2f(0.5,-1.0),Point2f(1.0,-1.0),Point2f( 1.5, -1),Point2f(   2,  -1),
                      Point2f( 2,-0.5),Point2f(  2,   0),Point2f(  2, 0.5),Point2f(   2,  1),Point2f( 1.5,   1),
                      Point2f( 1,   1),Point2f(  0.5, 1),Point2f(  0,   1),Point2f(-0.5,  1),Point2f(-1.0,   1),
                      Point2f(-1.5, 1),Point2f( -2,  1), Point2f( -2, 0.5),Point2f(  -2,  0),Point2f(  -2,-0.5),
                      Point2f( -2, -1),Point2f(-1.5, -1),Point2f( -1,  -1),Point2f(-0.5, -1),
                      Point2f(0,   -d),Point2f( d,    0),Point2f(  0,   d),Point2f(  -d,  0),Point2f(   1,  -d),
                      Point2f(1+d,  0),Point2f( 1,    d),Point2f(1-d,   0),Point2f(  -1, -d),Point2f(-1+d,   0),
                      Point2f( -1,  d),Point2f(-1-d,  0)};
int lm_indice[3] = {};
Pose2D pose_TG;
char contador=0, indice_pasado = 0;
bool flag = 0;
ros::Publisher pose_pub;
Mat image3;

  void timerCallback(const ros::TimerEvent&)
  {  
    if (flag)
    {
      if (lambda[0] < lambda[2])
        lambda_31 = 2*pi + (lambda[0] - lambda[2]);
      else
        lambda_31 = lambda[0] - lambda[2];
      if (lambda[0] > lambda[1])
        lambda_12 = 2*pi + (lambda[1] - lambda[0]);
      else 
        lambda_12 = lambda[1] - lambda[0];

      phi = atan2(landmark[lm_indice[0]].y - landmark[lm_indice[1]].y, landmark[lm_indice[0]].x - landmark[lm_indice[1]].x);
      sigma = pi - (atan2(landmark[lm_indice[0]].y - landmark[lm_indice[2]].y, landmark[lm_indice[0]].x - landmark[lm_indice[2]].x)) + phi;
      gamma_tg = sigma - lambda_31;
      
      long_31 = sqrt(pow(landmark[lm_indice[2]].x - landmark[lm_indice[0]].x,2) + pow(landmark[lm_indice[2]].y - landmark[lm_indice[0]].y,2));
      long_12 = sqrt(pow(landmark[lm_indice[1]].x - landmark[lm_indice[0]].x,2) + pow(landmark[lm_indice[1]].y - landmark[lm_indice[0]].y,2));

      //cout << "L12: " << long_12 << ' ' << "L31: "<<long_31 << endl; 
      tau = atan2(sin(lambda_12)*(long_12*sin(lambda_31)-long_31*sin(gamma_tg)),(long_31*sin(lambda_12)*cos(gamma_tg)-long_12*cos(lambda_12)*sin(lambda_31)));

      if (lambda_12 < pi && tau < 0)
        tau = tau + pi;
      else if (lambda_12 > pi && tau > 0)
        tau = tau - pi; 



      if (abs(sin(lambda_12)) > abs(sin(lambda_31)))
        long_1 = (long_12*sin(tau + lambda_12))/(sin(lambda_12));
      else
        long_1 = (long_31*sin(tau + sigma - lambda_31))/(sin(lambda_31));

      //cout << "L1: " << long_1 << " phi: " << phi*180/pi << " sigma: " << sigma*180/pi << " tau: " << tau*180/pi << endl;  
      //cout << " phi: " << phi*180/pi << " tau: " << tau*180/pi << "Lambda1: "<< lambda[0]*180/pi << endl;  
      X_r = landmark[lm_indice[0]].x - long_1*cos(phi + tau);
      Y_r = landmark[lm_indice[0]].y - long_1*sin(phi + tau);
      Theta_r = phi + tau - lambda[0];

      if (Theta_r > pi)
        Theta_r = Theta_r - 2*pi;
      else if(Theta_r <= -pi)
        Theta_r = Theta_r + 2*pi;

      if (abs(Theta_r) < 0.00001)
        Theta_r = 0;

      //cout << X_r << ' ' << Y_r << ' ' << Theta_r*180/pi<< endl;
      pose_TG.x = X_r;
      pose_TG.y = Y_r;
      pose_TG.theta = Theta_r*180/pi;

     
    }
    else
    {
      pose_TG.x = pose_TG.x;
      pose_TG.y = pose_TG.y;
      pose_TG.theta = pose_TG.theta;
    }
    pose_pub.publish(pose_TG);
    ros::spinOnce();
    //cout << "Pose: " << pose_TG << endl;
  }
  void chatterCallback(const VectorAruco::ConstPtr& landmarks_vistas)
  {
    
     if (landmarks_vistas->PoseAruco.size() >= 3)
     {
      for (int i = 0; i<3 ; i++)
      {
        lambda[i] = landmarks_vistas->PoseAruco[i].theta; 
        lm_indice[i] = landmarks_vistas->PoseAruco[i].id;
        //cout << "landmark seleccionada " << lm_indice[i] << ' ' << lambda[i]*360/(2*pi)<<endl;
      }
      flag = 1;
    }
    else
    {
      flag = 0;
    }   
  }
    
int main(int argc, char** argv)
{
  ros::init(argc, argv, "Triangulacion_Geometrica");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("Alg_TG", 10, chatterCallback);
  pose_pub = nh.advertise<Pose2D>("Pose_TG", 10);
  ros::Timer timer =  nh.createTimer(ros::Duration(1.0/30.0),timerCallback);
  ros::spin();
  return 0;
}



