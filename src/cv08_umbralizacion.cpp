#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
// ------------- Declaración de variables ------------- //
cv::Mat image, image_;
std::string r1 = "/home/kiqmc10/imgtest/sem.jpg";
std::string r2 = "/home/kiqmc10/imgtest/binary_image.jpg";
int tolerancia = 10;
cv::Vec3b referencia = 128;
cv::VideoCapture cap;
int sel = 0;
    
void timerCallback(const ros::TimerEvent&)
  { // Inicialización de la matriz que almacenará la imagen umbralizada
    image_ = cv::Mat::zeros(image.rows, image.cols, CV_8U);

    for (int y = 0; y < image_.rows; y++)
    {
      for(int x = 0; x<image_.cols ;x++)
      { 
        /* En las siguientes líneas se analiza la intensidad de cada pixel y se calcula la norma
          respecto al pixel de referencia, si es menor a la tolerancia se asigna 255 y si es mayor se asigna cero.*/
        cv::Vec3b pixel = image.at<cv::Vec3b>(y,x);
        uchar& pixel_   = image_.at<uchar>(y,x);

        if (cv::norm(pixel,referencia)<tolerancia)
          pixel_= 255;
        else
          pixel_ = 0;

      }
    }
    // Se muestra la imagen umbralizada
    cv::imshow("Window_", image_);
    cv::imshow("Window", image);
    cv::waitKey(3);

  }
void onTrackbarSlide( int pos, void *) {}

void my_mouse_callback( int event, int x, int y, int flags, void* param) 
  {
    /*Se detecta el evento de pulsar en la imagen de prueba*/
    cv::Mat& image = *(cv::Mat*) param;
    if (event == cv::EVENT_LBUTTONUP)
      {
        // Almacenamiento de la intesidad del pixel seleccionado para usar como referencia
        referencia = image.at<cv::Vec3b>(y,x); 
      }
  }
void my_mouse_callback1( int event, int x, int y, int flags, void* param) 
  {
    if (event == cv::EVENT_LBUTTONUP)
      {
        cv::imwrite(r2,image_);
      }
  }

int main(int argc, char** argv)
{
  //Inicialización del nodo
  ros::init(argc, argv, "cv08_umbralizacion");
  ros::NodeHandle nh;
  
  image = cv::imread(r1, cv::IMREAD_COLOR);
  
  // Se configura el nombre de la ventana en donde se mostrará el video
  cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);	 
  cv::namedWindow("Window_", cv::WINDOW_AUTOSIZE);  
  // Configuración de la trackbar para ajustar toleracia.
  cv::createTrackbar("Tolerancia", "Window_", &tolerancia, 255 ,onTrackbarSlide);   
  // Lectura de la imagen de prueba
  
  cv::imshow("Window", image); // Impresión de la imagen de prueba
  cv::waitKey(3);
  // Configuración del mouseCallBack
  cv::setMouseCallback( "Window",  my_mouse_callback,(void*)&image);
  cv::setMouseCallback( "Window_",  my_mouse_callback1,(void*)&image);

  if (image.empty()) 
    {
      ROS_INFO("could not read image");
      return 1;
    }
  // Configuración del timer a 24 FPS
  ros::Timer timer = nh.createTimer(ros::Duration(1.0/24.0),timerCallback);
  
  ros::spin();
}


