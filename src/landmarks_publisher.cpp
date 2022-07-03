#include "ros/ros.h"
#include "curso_pdi/TG_message.h"
#include "curso_pdi/TG_vector.h"
#include <geometry_msgs/Pose2D.h>
#include <sstream>
#include <random>
#define pi 3.141592653589793
std::random_device dev;
std::mt19937 rng(dev());
std::uniform_int_distribution<std::mt19937::result_type> dist6(0,50); 
using namespace std;
using namespace curso_pdi;
using namespace geometry_msgs;
const int num_lm = 3;
TG_vector landmarks; 
float lambdas[3] ={333.43, 251.57, 293.2}; 

void generador_landmarks(){

  landmarks.p_i.resize(3);

  for (int i = 0; i < num_lm; i++)
  {
    landmarks.p_i[i].id = i;
    landmarks.p_i[i].landmark.x = dist6(rng);
    landmarks.p_i[i].landmark.y = dist6(rng);
    landmarks.p_i[i].landmark.theta = lambdas[i]*2*pi/360.0;;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "landmarks_publisher");
  generador_landmarks();
  
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<TG_vector>("Alg_TG", 1);
  int c=0;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {

    chatter_pub.publish(landmarks);

    

    ros::spinOnce();
    if (c == 10) c = 0;
    loop_rate.sleep();
  }


  return 0;
}