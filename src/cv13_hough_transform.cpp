#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
using namespace cv;
using namespace std;
	//Creación de variable tipo videocapture. Esta almacena las escenas de la fuente seleccionada
  cv::VideoCapture cap;
  
  cv::Mat image, img_s, img_c, dst, cdst,cdstP;
  std::string r1 = "/home/kiqmc10/imgtest/img_p/img1.jpg";
  int Selector = 1;
  int umbral = 118;
  int rho = 1;
  int theta = 180;
  
  // Función para detección de borden con algoritmo de Canny
  void Canny_process(cv::Mat &image_canny)
  {
    cv::GaussianBlur(image_canny, image_canny, cv::Size(1 + 2*Selector,1 + 2*Selector), 0,0,cv::BORDER_DEFAULT);
    cv::Canny(image_canny,image_canny,100,200);
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
    // Lectura de la imágen de pueba
    img_s = cv::imread(r1,cv::IMREAD_GRAYSCALE );
    img_c = cv::imread(r1,cv::IMREAD_GRAYSCALE );
    image = cv::imread(r1,-1 );
    // Aplicación de filtro gaussiano para eliminación de ruido
    GaussianBlur(image, image, cv::Size(1 + 2*Selector,1 + 2*Selector), 0,0,cv::BORDER_DEFAULT);
    // Aplicación del algoritmo de canny para detección de bordes
    Canny_process(img_c);
    // Conversión de escala de grises a BGR
    cvtColor(img_s, cdst, COLOR_GRAY2BGR);
    cdstP = cdst.clone();
    // Proceso para detección de líneas con la transformada de Hough
    vector<cv::Vec2f> hough_L;
    HoughLines(img_c, hough_L,  1 + rho, CV_PI/(360 - theta), umbral, 0, 0 ); 
    draw_line(hough_L);
    // Impresión de las imágenes procesadas
    imshow("Stand Det", cdst);
    imshow("Canny", img_c);
    imshow("Window",image);
    waitKey(5);
    }

    void onTrackbarSlide( int pos, void *) {}
  
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cv_template_video");
  ros::NodeHandle nh;
  
  // Se configura el nombre de la ventana en donde se mostrarán las imágenes 
  cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Canny", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Stand Det", cv::WINDOW_AUTOSIZE);
  // Configuración de la trackbar para ajustar toleracias y modificar parámetros de funciones.
  cv::createTrackbar("GaussianBlur", "Window", &Selector, 5 ,onTrackbarSlide);
  cv::createTrackbar("Min num intersecciones", "Stand Det", &umbral, 1000 ,onTrackbarSlide); 
  cv::createTrackbar("Rho", "Stand Det", &rho, 100 ,onTrackbarSlide); 
  cv::createTrackbar("theta ", "Stand Det", &theta, 360 ,onTrackbarSlide);
  // Configuración de TImer para ejecutar los callbacks
  ros::Timer timer =  nh.createTimer(ros::Duration(1.0/60.0),timerCallback);
  ros::spin();
}


  
