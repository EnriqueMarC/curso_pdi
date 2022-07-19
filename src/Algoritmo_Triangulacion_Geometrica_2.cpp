#include <ros/ros.h>
#include "curso_pdi/VectorAruco.h"
#include <curso_pdi/PoseAruco.h>
#include <opencv2/highgui/highgui.hpp>
#include "cmath"
#include <geometry_msgs/Pose2D.h>
#include <iostream>
#include <fstream>
#define pi 3.141592653589793
using namespace std;
using namespace cv;
using namespace geometry_msgs;
/*-------------   Variables -----------------*/
double lambda_12 = 0, lambda_31 = 0, phi = 0, sigma = 0, gamma_tg = 0, long_12 = 0, long_31 = 0, tau = 0,long_1 = 0, X_r = 0, Y_r = 0, Theta_r = 0;
double lambda[3] = {};
float d = 0.5*10;
float scale = 0.60;
/*Point2f landmark[] = {Point2f( -0.0166,-0.605), Point2f(0.2889,  -0.62), Point2f(0.5941,-0.61), Point2f( 0.9023, -0.61), Point2f(1.215,  -0.6571), // 0 - 4
                      Point2f( 1.215,-0.3347),Point2f(  1.215,   -0.0243),Point2f(  1.215, 0.2838),Point2f(   1.215, 0.5959 ),Point2f( 0.9126,   0.6025), // 5 - 9
                      Point2f( 0.6004,   0.6025),Point2f(  0.
                      3308, 0.605),Point2f(  0.0161,   0.605),Point2f(-0.3005,  0.605),Point2f(-1.0*scale,   1*scale), // 10 - 14
                      Point2f(-1.5*scale, 1*scale),Point2f( -1.205,  0.6755), Point2f( -1.205, 0.3561),Point2f(  -1.205,  0.0454),Point2f(  -1.205, -0.2693), //15 - 19
                      Point2f( -1.205, -0.5778),Point2f(-1.5*scale, -1*scale),Point2f( -1*scale,  -1*scale),Point2f(-0.3386, -0.605), // 20 - 23
                      Point2f(0*scale,   -d),Point2f( d,    0*scale),Point2f(  0*scale,   d),Point2f(  -d,  0*scale),Point2f(   1*scale,  -d),
                      Point2f(1*scale+d,  0*scale),Point2f( 1*scale,    d),Point2f(1*scale-d,   0*scale),Point2f(  -1*scale, -d),Point2f(-1*scale+d,   0*scale),
                      Point2f( -1*scale,  d),Point2f(-1*scale-d,  0*scale)};*/
Point2f landmark[] = {Point2f( 0 ,- 0.60), Point2f(0.304,  -0.60), Point2f(0.6078,-0.60), Point2f( 0.908188, -0.60031), Point2f(1.2227,  -0.659194), // 0 - 4
                      Point2f( 1.22882,-0.341989),Point2f(  1.22117,   0.2758),Point2f(  1.23632, 0.284565),Point2f(   1.24764, 0.608595 ),Point2f( 0.90,   0.605), // 5 - 9
                      Point2f( 0.604,   0.605),Point2f(  0.30, 0.605),Point2f(  0.0,   0.605),Point2f(-0.298,  0.605),Point2f(-0.599,   0.605), // 10 - 14
                      Point2f(-0.899, 0.61),Point2f( -1.205,  0.61), Point2f( -1.205, 0.305),Point2f(  -1.205,  0.01),Point2f(  -1.205, -0.295), //15 - 19
                      Point2f( -1.205, -0.595),Point2f(-0.90, -0.60),Point2f( -0.604,  -0.60),Point2f(-0.314154, -0.6004), // 20 - 23
                      Point2f(0*scale,   -d),Point2f( d,    0*scale),Point2f(  0*scale,   d),Point2f(  -d,  0*scale),Point2f(   1*scale,  -d),
                      Point2f(1*scale+d,  0*scale),Point2f( 1*scale,    d),Point2f(1*scale-d,   0*scale),Point2f(  -1*scale, -d),Point2f(-1*scale+d,   0*scale),
                      Point2f( -1*scale,  d),Point2f(-1*scale-d,  0*scale)};

int lm_indice[3] = {};
Pose2D pose_TG;
char contador=0, indice_pasado = 0;
bool flag = 0;
ros::Publisher pose_pub;
 //poseAruco char id, float64 x, y, theta vectorArucos poseAruco[] vector
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
      pose_TG.theta = Theta_r;     
    }
    else
    {
      pose_TG.x = pose_TG.x;
      pose_TG.y = pose_TG.y;
      pose_TG.theta = pose_TG.theta;
    }
    pose_pub.publish(pose_TG);
    //cout << "Pose: " << pose_TG << endl;
    //cout << "Lambdas" << lm_indice[0]<< ' ' << lm_indice[1] << ' '<<lm_indice[2] << endl;
  }
  void chatterCallback(const triangulacion::VectorAruco::ConstPtr& landmarks_vistas)
  {
    //cou << "DATA RECEIVED" << endl;
    //cout <<landmarks_vistas->aruco[0].theta<<endl;
    //cout <<landmarks_vistas->aruco[1].id<<endl;
    //cout <<landmarks_vistas->aruco[0].pitch*180/pi<<endl;
    //cout <<landmarks_vistas->aruco[1].pitch<<endl;
    //landmarks_vistas.aruco.[1] x 
    //landmarks_vistas.aruco.[2] y
    //landmarks_vistas.aruco.[3] pitch
		
		//cout << "Detección";
    
     if (landmarks_vistas->aruco.size() >= 3)
     {
      for (int i = 0; i<3 ; i++)
      {
        lambda[i] = landmarks_vistas->aruco[i].theta * pi/180.0; 
		if (lambda[i] < 0)
			lambda[i] = 2*pi + lambda[i];

        lm_indice[i] = landmarks_vistas->aruco[i].id; 

	//cout <<"landmark "<< lm_indice[i] << " angulo: " << lambda[i]*360/(2*pi) << endl;
      }
      flag = 1;
    }
    else
    {
      flag = 0;
    } 
//		lambda[0]=1.56264-pi/2;
//		lm_indice[0]=0;
//		lambda[1]=1.305496-pi/2;
//		lm_indice[1]=1;
//		lambda[2]=1.00622-pi/2;
//		lm_indice[2]=2;
  }
    
int main(int argc, char** argv)
{
  ros::init(argc, argv, "Triangulacion_Geometrica");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("Alg_TG", 1000, chatterCallback);
	pose_pub = nh.advertise<Pose2D>("Pose_TG", 10);
  /*for (int i = 0; i < 36;i++)
	{
		landmark[i] = Point2f(landmark[i].x*scale + 0.015,landmark[i].y*scale );
	}*/
  ros::Timer timer =  nh.createTimer(ros::Duration(1.0/15.0),timerCallback);
  ros::spin();
  return 0;
}



