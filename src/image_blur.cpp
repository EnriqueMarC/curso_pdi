#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
	std::string ruta;
  cv::Mat image;
  cv::Mat out;
  
  int Selector = 0;
  int SSel = 0;
  
  void timerCallback(const ros::TimerEvent&){  
    
  	image = cv::imread(ruta, -1);
  	
		cv::GaussianBlur(image, image, cv::Size(1 + 2*Selector,1 + 2*Selector), SSel,0,cv::BORDER_DEFAULT);
  	
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
  ros::init(argc, argv, "cv_template");
  ros::NodeHandle nh;
  ROS_INFO("Start cv_template_file");
  
  // Se configura el nombre de la ventana en donde se mostrará el video
  cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);	
  

  ros::Timer timer =    		nh.createTimer(ros::Duration(1.0/24.0),timerCallback);
  
  cv::createTrackbar("Blur Level", "Window", &Selector, 100 ,onTrackbarSlide);
  
  cv::createTrackbar("SSblur", "Window", &SSel, 100 ,onTrackbarSlide);
  
  ros::spin();
}


