#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>

	//Creación de variable tipo videocapture. Esta almacena las escenas de la fuente seleccionada
  cv::VideoCapture cap;
  
  cv::Mat image, element;
  int modo = 0, tipo = 0, size = 0, erosion_type;
  cv::Mat flipimg;
  std::string r1 = "/home/kiqmc10/imgtest/binary_image.jpg";
  std::string r2 = "/home/kiqmc10/imgtest/binary_image_.jpg";
  int Selector = 0;
  bool ready = true;
  
  void onTrackbarSlide( int pos, void *) {
		 // Para este caso no se hace nada con el llamado de la función
	}
	
  void timerCallback(const ros::TimerEvent&){  
    // La función flip invierte la imagen con respecto a la vertical
    if (ready)
    image = cv::imread(r1,cv::IMREAD_GRAYSCALE);
    else
    image = cv::imread(r2,cv::IMREAD_GRAYSCALE);
    

    element = cv::getStructuringElement( tipo, cv::Size( 2*size + 1, 2*size+1 ),
                       cv::Point( size, size ) );
    if (modo) cv::dilate(image, image,element);
    else cv::erode(image, image,element);

    
    if (image.empty()) 
	  {
	    ROS_INFO("could not read image");
	    return; 
	  }
	  // Se muestra la imagen capturada
    cv::imshow("Window", image);
    cv::waitKey(5);
    }
  void my_mouse_callback( int event, int x, int y, int flags, void* param) 
  {
    /*Se detecta el evento de pulsar en la imagen de prueba*/
    cv::Mat& image = *(cv::Mat*) param;
    if (event == cv::EVENT_LBUTTONUP)
      {
        cv::imwrite(r2,image);
        ready = false;
      }
  }
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cv_template_video");
  ros::NodeHandle nh;
  
  // Se configura el nombre de la ventana en donde se mostrará el video
  cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);
  cv::createTrackbar("Modo", "Window", &modo, 1 ,onTrackbarSlide);
  cv::createTrackbar("Tipo de conexión", "Window", &tipo, 2 ,onTrackbarSlide);
  cv::createTrackbar("Tamaño", "Window", &size, 20 ,onTrackbarSlide);
	cv::setMouseCallback( "Window",  my_mouse_callback,(void*)&image);
	// Si no se detecta la cámara el nodo retorna un 1 y finaliza. Escribe en la terminal que no fue posible abrir la cámara.
 
  ros::Timer timer =    nh.createTimer(ros::Duration(1.0/60.0),timerCallback);
  ros::spin();
}



