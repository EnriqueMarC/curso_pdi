#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
using namespace cv;
using namespace std;
	//Creación de variable tipo videocapture. Esta almacena las escenas de la fuente seleccionada
  cv::VideoCapture cap;
  
  cv::Mat image, image_gray;
  std::string r1 = "/home/kiqmc10/imgtest/img_p/img5.jpg";
  int Selector = 2;
  int dist = 5;
  int min_radio = 2;
  int max_radio = 9;
  int canny = 200;
  int umbral_center = 65;
  int depth = 1;
  
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
	// Función para realizar el cálculo de los bordes con Canny
  void Canny_process(cv::Mat &image_canny)
  {
    medianBlur(image_canny, image_canny, 1 + 2*Selector);
    cv::Canny(image_canny,image_canny,canny/2,canny,3); // Aplicación del algoritmo canny a una imagen en escala de grises
    return;
  }

  void timerCallback(const ros::TimerEvent&)
  {  
    image_gray = cv::imread(r1,cv::IMREAD_GRAYSCALE );
    image = cv::imread(r1,-1 );

    Mat canny_img, g_i;
    canny_img = cv::imread(r1,cv::IMREAD_GRAYSCALE ); 
    g_i = cv::imread(r1,cv::IMREAD_GRAYSCALE ); 
    Canny_process(canny_img);
    medianBlur(image, image, 1 + 2*Selector);
    cvtColor(image, image_gray, COLOR_BGR2GRAY);

    
    vector<Vec3f> circles;
    HoughCircles(image_gray, circles, HOUGH_GRADIENT, 2,
                 image_gray.rows/(1.0 + dist),  // change this value to detect circles with different distances to each other
                 canny, 30,min_radio , max_radio // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );
    cvtColor(canny_img, image_gray, COLOR_GRAY2BGR);
    cvtColor(g_i, g_i, COLOR_GRAY2BGR);
    compute_circles(circles, image_gray);
    compute_circles(circles, g_i);

    cv::imshow("Círculos detectados", image_gray);
    cv::imshow("Gray image",g_i);
    cv::imshow("Window",image);
    cv::waitKey(5);
    }

    void onTrackbarSlide( int pos, void *) {}
  
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cv_template_video");
  ros::NodeHandle nh;
  
  // Se configura el nombre de la ventana en donde se mostrará el video
  cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Círculos detectados", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Gray image", cv::WINDOW_AUTOSIZE);
  // Configuración de la trackbar para ajustar toleracia.
  cv::createTrackbar("GaussianBlur", "Window", &Selector, 20 ,onTrackbarSlide);
  cv::createTrackbar("Distancia", "Círculos detectados", &dist, 500 ,onTrackbarSlide); 
  cv::createTrackbar("Radio mínimo", "Círculos detectados", &min_radio, 255 ,onTrackbarSlide); 
  cv::createTrackbar("Radio máximo ", "Círculos detectados", &max_radio, 255 ,onTrackbarSlide);
  cv::createTrackbar("Umbral canny ", "Círculos detectados", &canny, 255 ,onTrackbarSlide);
  //cv::createTrackbar("Umbral centro ", "Círculos detectados", &umbral_center, 200 ,onTrackbarSlide);
  //cv::createTrackbar("Depth ", "Círculos detectados", &depth, 100 ,onTrackbarSlide);
  image = cv::imread(r1,cv::IMREAD_GRAYSCALE );
  cv::imshow("Window",image);
  ros::Timer timer =  nh.createTimer(ros::Duration(1.0/60.0),timerCallback);
  ros::spin();
}



