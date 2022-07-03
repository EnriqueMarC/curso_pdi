#include "ros/ros.h" // Principal
#include "geometry_msgs/Pose2D.h"
#include <math.h>       /* atan2 */

#define PI 3.14159265

ros::Publisher pos_referencia;
geometry_msgs::Pose2D P_msg, P_;
double t0;//Se declara la variable t0 como global

const float T[] = {0, 5, 15, 23, 43, 51, 61};//Arreglo de tiempos
const float X[] = {0, 0,  1,  1, -1, -1,  0};//Arreglo de puntos en X
const float Y[] = {0, 0,  0,  1,  1,  0,  0};//Arreglo de puntos en Y
const float M[] = {0, 1,  1,  2,  1,  2,  1};//Modos seleccionados
const int pT = 7;//Número de puntos


void timerCallback(const ros::TimerEvent&);//Función que se manda a llamar cada cierto tiempo (indicado en el timer)

geometry_msgs::Pose2D seg_lineal(float ti, float tf, float xi, float xf, float yi, float yf, float t);
geometry_msgs::Pose2D seg_med_cir(float ti, float tf, float xi, float xf, float yi, float yf, float t);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "deslizador");
  ros::NodeHandle n;
  pos_referencia = n.advertise<geometry_msgs::Pose2D>("Pose_ref", 10);
  
  t0 = ros::Time::now().toSec();
  
  P_.x = 0;//Punto origen en x
  P_.y = 0;//Punto origen en y
  P_.theta = 0;//Orientación inicial
  
  ros::Timer timer = n.createTimer(ros::Duration(0.1),timerCallback);//Pide  la duración, y el nombre de la función que se va a mandar a llamar según el tiempo escogido.
  ros::spin();

  return 0;
}

// Generación de trayectorias

void timerCallback(const ros::TimerEvent&){

        double t = ros::Time::now().toSec() - t0;
        
        if(t <= T[pT-1]){
         	int k=1;         
         	while (k < pT){
         		if(T[k-1] <= t && t <= T[k])
         			break;
         		k++;	
         	}
         	
        	if(M[k]==1) 
        		P_msg = seg_lineal(T[k-1],T[k],X[k-1],X[k],Y[k-1],Y[k],t);
        	else
        		P_msg = seg_med_cir(T[k-1],T[k],X[k-1],X[k],Y[k-1],Y[k],t);
         	
        }
        else{
 		P_msg.x = X[pT-1];
 		P_msg.y = Y[pT-1];   
 		   
        }
        
	if(sqrt(pow(P_msg.y-P_.y,2.0)+pow(P_msg.x-P_.x,2.0))/2.0 > 0)
		P_msg.theta = atan2(P_msg.y-P_.y,P_msg.x-P_.x);
	else
		P_msg.theta = P_.theta;
	
	pos_referencia.publish(P_msg);
	P_ = P_msg;
}
// Modo segmento de recta
geometry_msgs::Pose2D seg_lineal(float ti, float tf, float xi, float xf, float yi, float yf, float t){
	geometry_msgs::Pose2D P;	
	P.x =(t-ti)*(xf-xi)/(tf-ti)+xi;
	P.y =(t-ti)*(yf-yi)/(tf-ti)+yi;
	return P;
}
//Modo segmento de medio circulo
geometry_msgs::Pose2D seg_med_cir(float ti, float tf, float xi, float xf, float yi, float yf, float t){
	geometry_msgs::Pose2D P;
	double r = sqrt(pow(yf-yi,2.0)+pow(xf-xi,2.0))/2.0;
	double a0 = atan2(yf-yi,xf-xi) + PI;
	double xc = (xf+xi)/2.0;
	double yc = (yf+yi)/2.0; 
	double a = (t-ti)*PI / (tf-ti) +a0;
	P.x = r*cos(a)+xc;
	P.y = r*sin(a)+yc;
	return P;
}
