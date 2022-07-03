#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>

float d_kernel[] = {1, 4, 7.5, 4,1,
                    4,16,26,16,4,
                    7,26,41,26,7,
                    4,16,26,16,4,
                    1, 4, 7, 4,1};
cv::Mat kernel(5,5,CV_32FC1,d_kernel);
cv::Point3f p;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cv_operaciones");
  ros::NodeHandle nh;

  // Si no se detecta la cámara el nodo retorna un 1 y finaliza. Escribe en la terminal que no fue posible abrir la cámara.
  
  p = cv::Point3f(3,5,6);

  p = kernel.at<float>(0,2) * p;

  std::cout << p << kernel.at<float>(0,2) << std::endl;


  // Si la cámara funciona correctamente se imprime en ROS_INFO que se ha comenzado a grabar
  
 
}