#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>

	//Creación de variable tipo videocapture. Esta almacena las escenas de la fuente seleccionada
  cv::VideoCapture cap;
  
  cv::Mat image;
  cv::Mat flipimg;
  int Selector = 0;
  
  void onTrackbarSlide( int pos, void *) {
		 // Para este caso no se hace nada con el llamado de la función
	}
	
  void timerCallback(const ros::TimerEvent&){  
    
    //Se realiza una lectura de la fuente seleccionada
    cap >> image;
    // La función flip invierte la imagen con respecto a la vertical
  	//cv::flip(image, flipimg,1);
  	//cv::GaussianBlur(flipimg, flipimg, cv::Size(1 + 2*Selector,1 + 2*Selector), 0,0,cv::BORDER_DEFAULT);
    if (image.empty()) 
	  {
	    ROS_INFO("could not read image");
	    return; 
	  }
	  // Se muestra la imagen capturada
    cv::imshow("Window", image);
    cv::waitKey(5);
    }
  
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cv_template_video");
  ros::NodeHandle nh;
  
  // Configuración de la fuente de video. El primer argumento selecciona la fuente y el segundo configura la forma en la que se interpreta la imagen. Con cv::CAP_ANY se le dice que lo haga de forma automática. 
  cap.open("/home/kiqmc10/imgtest/img_p/video2.m4v");
  
  // Se configura el nombre de la ventana en donde se mostrará el video
  cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);
  
  cv::createTrackbar("Blur Level", "Window", &Selector, 100 ,onTrackbarSlide);
	
	// Si no se detecta la cámara el nodo retorna un 1 y finaliza. Escribe en la terminal que no fue posible abrir la cámara.
  if (!cap.isOpened())  
  {
      ROS_INFO("ERROR! Unable to open camera");
      return 1;
  }
  
  // Si la cámara funciona correctamente se imprime en ROS_INFO que se ha comenzado a grabar
  ROS_INFO("Start grabbing...");
  ros::Timer timer =    nh.createTimer(ros::Duration(1.0/40.0),timerCallback);
  ros::spin();
}



