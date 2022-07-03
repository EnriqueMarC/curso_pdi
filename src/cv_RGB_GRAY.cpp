#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
	std::string ruta;
  cv::Mat image;
  int Selector = 0;
  int imcolor = 0;
  
  void timerCallback(const ros::TimerEvent&){  
    if (Selector)
    image = cv::imread(ruta, cv::IMREAD_GRAYSCALE);
    else
  	image = cv::imread(ruta, -1);
  	
  	
    if (image.empty()) 
    {
      ROS_INFO("could not read image");
      return;
    }
    cv::imshow("Window", image);
    cv::waitKey(33);
    }
   void onTrackbarSlide( int pos, void *) {
		 // Para este caso no se hace nada con el llamado de la función
	}
  
int main(int argc, char** argv)
{
	ruta = argv[1];
  ros::init(argc, argv, "CV_RGB_GRAY");
  ros::NodeHandle nh;
  ROS_INFO("Start cv_template_file");
  
  // Se configura el nombre de la ventana en donde se mostrará el video
  cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);	
  

  ros::Timer timer =    		nh.createTimer(ros::Duration(1.0/24.0),timerCallback);
  cv::createTrackbar("RGB-GS", "Window", &Selector, 1 ,onTrackbarSlide);
  ros::spin();
}


