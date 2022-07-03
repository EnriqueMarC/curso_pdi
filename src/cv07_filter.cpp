#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <cmath>

	//Creación de variable tipo videocapture. Esta almacena las escenas de la fuente seleccionada
  cv::VideoCapture cap; 
  cv::Mat image, x_image, y_image, xy_image, kernel;
  cv::Mat flipimg;
  std::string r1 = "/home/kiqmc10/imgtest/edificio2.jpg";
  cv::Point2i p;
  // Segmentar separar en dos categorías. 
  void onTrackbarSlide( int pos, void *) {
		 // Para este caso no se hace nada con el llamado de la función
	}	
  void timerCallback(const ros::TimerEvent&){
    
    //Se realiza una lectura de la fuente seleccionada
    cap.read(image);
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    xy_image = cv::Mat::zeros(image.rows,image.cols,CV_8U);
    //std::cout << image.type()<<std::endl;
    // La función flip invierte la imagen con respecto a la vertical
  	cv::flip(image, image,1);
    //image = cv::imread(r1, cv::IMREAD_COLOR);
    //kernel = (cv::Mat_<double>(3,3)<< 0, -1, 0, -1, 5, -1, 0, -1, 0); //Filtro Sharpen
    //kernel = (1./9)*cv::Mat::ones(21,21,CV_32F); //Filtro promediador
    kernel = (cv::Mat_<double>(3,3)<< -1, 0, 1, -2, 0, 2, -1, 0, 1); // Derivada en x
    cv::filter2D(image,x_image,-1,kernel);
    //kernel = (cv::Mat_<double>(3,3)<< -1, -2, -1, 0, 0, 0, 1, 2, 1); // Derivada en y
  	cv::filter2D(image,y_image,-1,kernel.t());
    
    for (int x = 0; x < image.cols; x++)
    {
      for(int y = 0; y < image.rows; y++)
      {
        //p.x = x_image.at<uchar>(y,x);
        //p.y = y_image.at<uchar>(y,x);
        //pixel = (uchar)(pow(pow(x_image.at<uchar>(y,x), 2.0)+ pow(y_image.at<uchar>(y,x),2.0),0.5)/pow(2.0,0.5));
        xy_image.at<uchar>(y,x) = cv::norm(cv::Point2i(x_image.at<uchar>(y,x),y_image.at<uchar>(y,x)));

      }
    }
    
    if (image.empty()) 
	  {
	    ROS_INFO("could not read image");
	    return; 
	  }
	  // Se muestra la imagen capturada
    cv::imshow("Derivada_X", x_image);
    cv::imshow("Derivada_Y", y_image);    
    cv::imshow("Gradiente",  xy_image);
    cv::waitKey(1);
    }
  
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cv_template_video");
  ros::NodeHandle nh;
  
  // Configuración de la fuente de video. El primer argumento selecciona la fuente y el segundo configura la forma en la que se interpreta la imagen. Con cv::CAP_ANY se le dice que lo haga de forma automática. 
  cap.open(0, cv::CAP_ANY);
  
  // Se configura el nombre de la ventana en donde se mostrará el video
  cv::namedWindow("Derivada_X", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Derivada_Y", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Gradiente", cv::WINDOW_AUTOSIZE);
	
	// Si no se detecta la cámara el nodo retorna un 1 y finaliza. Escribe en la terminal que no fue posible abrir la cámara.
  if (!cap.isOpened())  
  {
      ROS_INFO("ERROR! Unable to open camera");
      return 1;
  }
  
  // Si la cámara funciona correctamente se imprime en ROS_INFO que se ha comenzado a grabar
  ROS_INFO("Start grabbing...");
  ros::Timer timer =    nh.createTimer(ros::Duration(1.0/60.0),timerCallback);
  ros::spin();
}



