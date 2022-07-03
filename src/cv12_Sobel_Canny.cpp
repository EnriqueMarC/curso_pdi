#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
using namespace cv;
	//Creación de variable tipo videocapture. Esta almacena las escenas de la fuente seleccionada
  cv::VideoCapture cap;
  
  cv::Mat image, img_s, img_c;
  std::string r1 = "/home/kiqmc10/imgtest/img_p/img1.jpg";
  int Selector = 1, u_max = 200, u_min = 100, imagen = 0;
  bool ready = false;
  
  // Función para realizar el cálculo de los bordes con Sobel
  void Sobel_process (cv::Mat &image_sobel)
  {
    cv::Mat h_dev, v_dev;
    cv::GaussianBlur(image_sobel, image_sobel, cv::Size(1 + 2*Selector,1 + 2*Selector), 0,0,cv::BORDER_DEFAULT); // Filtro Gaussiano para eliminación de ruido
    cv::Sobel(image_sobel, h_dev,CV_32F,1,0); // Cálculo de la derivada en x
    cv::Sobel(image_sobel, v_dev,CV_32F,0,1); // Cálculo de la derivada en y
    convertScaleAbs(h_dev, h_dev); // conversión de los valores de las derivadas parciales a su equivalente en 8 bits sin signo
    convertScaleAbs(v_dev, v_dev);
    addWeighted(h_dev, 0.5, v_dev, 0.5, 0, image_sobel); // Aproximación del gradiente con una suma ponderada. 
    return;
  }
  // Función para realizar el cálculo de los bordes con Canny
  void Canny_process(cv::Mat &image_canny)
  {
    cv::GaussianBlur(image_canny, image_canny, cv::Size(1 + 2*Selector,1 + 2*Selector), 0,0,cv::BORDER_DEFAULT); // Filtro Gaussiano para eliminación de ruido
    cv::Canny(image_canny,image_canny,u_min,u_max,3); // Aplicación del algoritmo canny a una imagen en escala de grises
    return;
  }
	
  void timerCallback(const ros::TimerEvent&){  
    if (ready){
      cap >> image;
      cvtColor(image,img_s,COLOR_BGR2GRAY);
      img_c = img_s.clone();
      // Filtro gaussiano a la imagen de prueba
      cv::GaussianBlur(image, image, cv::Size(1 + 2*Selector,1 + 2*Selector), 0,0,cv::BORDER_DEFAULT);
      Sobel_process(img_s); // llamada de la función sobel
      Canny_process(img_c); // llamada de la función canny
      }
      // Impresión de las imágenes obtenidas del procesamiento. 
      cv::imshow("Sobel", img_s); 
      cv::imshow("Canny", img_c);
      cv::imshow("Window",image);
    
    
    cv::waitKey(5);
    }

  void onTrackbarSlide( int pos, void *) {}

  void my_mouse_callback( int event, int x, int y, int flags, void* param) 
  {
      if (event == cv::EVENT_LBUTTONUP)
      { 
        ready = true; // Activación de bandera para inicio de algoritmo k-means
      }
  }
int main(int argc, char** argv)
{
  // Inicialización del nodo.
  ros::init(argc, argv, "cv_template_video");
  ros::NodeHandle nh;
  
  cap.open("/home/kiqmc10/imgtest/img_p/video2.m4v");
  if (!cap.isOpened())  
  {
      ROS_INFO("ERROR! Unable to open camera");
      return 1;
  }
  cap >> image;
  // Se configura el nombre de la ventana en donde se mostrará el video
  cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Sobel", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Canny", cv::WINDOW_AUTOSIZE);
  image = cv::Mat::zeros(image.rows, image.cols, image.type());
  img_s = cv::Mat::zeros(image.rows, image.cols, image.type());
  img_c = cv::Mat::zeros(image.rows, image.cols, image.type());
  // Creación de trackbars para modificación de parámetros de las funciones
  cv::createTrackbar("GaussianBlur", "Window", &Selector, 5 ,onTrackbarSlide); 
  cv::createTrackbar("Umbral min", "Canny", &u_min, 255 ,onTrackbarSlide); 
  cv::createTrackbar("Umbral máx", "Canny", &u_max, 255 ,onTrackbarSlide); 
  // Creación del callback del mouse
  cv::setMouseCallback( "Window",  my_mouse_callback,(void*)&image);
  
  ros::Timer timer =  nh.createTimer(ros::Duration(1.0/60.0),timerCallback);
  ros::spin();
}



