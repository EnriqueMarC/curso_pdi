#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
using namespace cv;
using namespace std;
	//Creación de variable tipo videocapture. Esta almacena las escenas de la fuente seleccionada
  cv::VideoCapture cap;
  
  cv::Mat image, image_p, img_s, img_c, cdst, image_gray, g_i;
  bool ready = false;
  int Selector = 5;  
  int Selector2 = 2;
  int umbral = 90;
  int rho = 0;
  int theta = 180;
  int dist = 5;
  int min_radio = 10;
  int max_radio = 20;
  int canny = 110;
  
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

  void compute_circles (vector<cv::Vec3f> &circles, Mat &image_gray)
  {
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Vec3i c = circles[i];
        Point center = Point(c[0], c[1]);
        // circle center
        circle( image_gray, center, 1, Scalar(255,0,0), 2, LINE_AA);
        // circle outline
        int radius = c[2];
        circle( image_gray, center, radius, Scalar(0,255,100), 2, LINE_AA);
    }
  }    

  void Canny_process(cv::Mat &image_canny)
  {
    GaussianBlur(image_canny, image_canny, cv::Size(1 + 2*Selector,1 + 2*Selector), 0,0,cv::BORDER_DEFAULT); // Filtro Gaussiano para eliminación de ruido     
    Canny(image_canny,image_canny,canny/2,canny,3); // Aplicación del algoritmo canny a una imagen en escala de grises
    return;
  }
  // Función para dibujar las líneas con los datos de r y theta encontrados en el algoritmo de HoughLInes
  void draw_line (vector<cv::Vec2f> &lines)
  {    
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        // Función para dibujar las líneas
        cv::line( cdst, pt1, pt2, Scalar(0,255,0), 1, LINE_AA);
    }
  }
	
  void timerCallback(const ros::TimerEvent&){  
    if (ready)
    {
      cap >> image;
      image_p = image.clone();
    }
    else
      image = image_p.clone();

      cvtColor(image, img_s, COLOR_BGR2GRAY);
      img_c = img_s.clone();
      g_i  = img_s.clone();
      image_gray = img_s.clone();
      // Aplicación de filtro gaussiano para eliminación de ruido
      medianBlur(image, image, 1 + 2*Selector2);
      // Aplicación del algoritmo de canny para detección de bordes
      Canny_process(img_c);
      // Conversión de escala de grises a BGR
      cvtColor(img_s, cdst, COLOR_GRAY2BGR);
      // Proceso para detección de líneas con la transformada de Hough
      vector<cv::Vec2f> hough_L;
      HoughLines(img_c, hough_L,  1 + rho, CV_PI/(360 - theta), umbral, 0, 0 ); 
      draw_line(hough_L);

      Sobel_process(img_s); // llamada de la función sobel

      GaussianBlur(image_gray, image_gray, cv::Size(1 + 2*Selector,1 + 2*Selector), 0,0,cv::BORDER_DEFAULT); // Filtro Gaussiano para eliminación de ruido     
      vector<Vec3f> circles;
      HoughCircles(image_gray, circles, HOUGH_GRADIENT, 2,image_gray.rows/(1.0 + dist),canny, 35,min_radio , max_radio);
      cvtColor(g_i, g_i, COLOR_GRAY2BGR);
      compute_circles(circles, g_i);
      

    imshow("Círculos detectados", g_i);
    imshow("Stand Det", cdst);
    imshow("Canny", img_c);
    imshow("Sobel", img_s);
    imshow("Window",image);
    waitKey(5);
    }

  void onTrackbarSlide( int pos, void *) {}

  void my_mouse_callback( int event, int x, int y, int flags, void* param) 
  {
      if (event == cv::EVENT_LBUTTONUP)
      { 
        ready = !ready; // Activación de bandera para inicio de algoritmo k-means
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
  
  image = cv::Mat::zeros(image.rows, image.cols, image.type());
  img_c = cv::Mat::zeros(image.rows, image.cols, image.type());
  img_s = cv::Mat::zeros(image.rows, image.cols, image.type());
  cdst = cv::Mat::zeros(image.rows, image.cols, image.type());
  g_i = cv::Mat::zeros(image.rows, image.cols, image.type());
  image_p = image.clone();
  cout << image.rows << endl;
   // Se configura el nombre de la ventana en donde se mostrarán las imágenes 
  namedWindow("Window", cv::WINDOW_AUTOSIZE);
  moveWindow("Window", 20,20);
  namedWindow("Canny", cv::WINDOW_AUTOSIZE);
  moveWindow("Canny", image.cols,20);
  namedWindow("Sobel", cv::WINDOW_AUTOSIZE);
  moveWindow("Sobel", 2*image.cols,20);
  namedWindow("Stand Det", cv::WINDOW_AUTOSIZE);
  moveWindow("Stand Det", 3*image.cols,20);
  namedWindow("Círculos detectados", cv::WINDOW_AUTOSIZE);
  moveWindow("Círculos detectados", 4*image.cols,20);
  // Configuración de la trackbar para ajustar toleracias y modificar parámetros de funciones.
  cv::createTrackbar("GaussianBlur", "Window", &Selector, 5 ,onTrackbarSlide);
  cv::createTrackbar("Min num intersecciones", "Stand Det", &umbral, 1000 ,onTrackbarSlide); 
  cv::createTrackbar("Rho", "Stand Det", &rho, 100 ,onTrackbarSlide); 
  cv::createTrackbar("theta ", "Stand Det", &theta, 360 ,onTrackbarSlide);
  cv::createTrackbar("medianBlur", "Window", &Selector2, 20 ,onTrackbarSlide);
  cv::createTrackbar("Distancia", "Círculos detectados", &dist, 500 ,onTrackbarSlide); 
  cv::createTrackbar("Radio mínimo", "Círculos detectados", &min_radio, 255 ,onTrackbarSlide); 
  cv::createTrackbar("Radio máximo ", "Círculos detectados", &max_radio, 255 ,onTrackbarSlide);
  cv::createTrackbar("Umbral canny ", "Círculos detectados", &canny, 255 ,onTrackbarSlide);
  // Creación del callback del mouse
  cv::setMouseCallback( "Window",  my_mouse_callback,(void*)&image);
  
  ros::Timer timer =  nh.createTimer(ros::Duration(1.0/30.0),timerCallback);
  ros::spin();
}



