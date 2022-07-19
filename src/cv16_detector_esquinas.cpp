#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/features2d.hpp>
//#include <opencv2/xfeatures2d/nonfree.hpp>
//#include <opencv2/xfeatures2d.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#define PI 3.14159265
using namespace cv;
using namespace std;
//Creación de variable tipo videocapture. Esta almacena las escenas de la fuente seleccionada
cv::VideoCapture cap;
cv::Mat image, image_gray;
int thresh = 180, thresh_fast = 110;
/*
void SURF_DETECTOR()
{
  Mat image_surf = image.clone();
  int minHessian = 400;
  cv::xfeatures2d::SurfFeatureDetector::create detector( 400);
  std::vector<KeyPoint> keypoints;
  detector.detect( image_surf, keypoints );
  
  Mat img_keypoints_surf; 
  drawKeypoints( image1, keypoints_1, img_keypoints_surf, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
 
  imshow("SURF DETECTOR", img_keypoints_surf );
  waitKey(0);

}
void SIFT_DETECTOR()
{
   cv::SiftFeatureDetector detector;
    std::vector<cv::KeyPoint> keypoints;
    detector.detect(image, keypoints);
    
    Mat image_sift = image.clone();
    cv::Mat output;
    cv::drawKeypoints(image, keypoints, output);
    cv::imshow("SIFT DETECTOR", output);
    
}*/

void HARRIS_DETECTOR( int, void* )
{
    //Conversión de color a escala de grises
    cvtColor( image, image_gray, COLOR_BGR2GRAY );
    int blockSize = 5; // Tamaño de la región alrededor del pixel
    int apertureSize = 3; // Parámetro de apertura para el operador sobel
    double k = 0.04; // Parámetro libre del detector Harris
    // Inicialización de imagen sobre la que se harán las operaciones para buscar esquinas
    Mat image_harris = image.clone();
    Mat dst = Mat::zeros( image.size(), CV_32FC1 );
    // Función para calcular los features con el método de Harris.
    cornerHarris( image_gray, dst, blockSize, apertureSize, k );
    Mat dst_norm;
    // Normalización de los datos calculados.
    normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    for( int i = 0; i < dst_norm.rows ; i++ )
    {
        for( int j = 0; j < dst_norm.cols; j++ )
        {
            if( (int) dst_norm.at<float>(i,j) > thresh )
            {
              // Dibujo de los círculos que señalan donde hay una esquina
                circle( image_harris, Point(j,i), 3,  Scalar(0,255,0), 1, 2, 0 );
            }
        }
    }
    // Impresión de la imagen con las esquinas calculadas
    imshow( "HARRIS DETECTOR", image_harris );
}

void FAST_DETECTOR ()
{
  // Creación de variables de tipo mat para hacer una copia de la imagen original
  Mat image_gray, image_fast;
  std::vector<cv::KeyPoint> keypoints; // Creación de un vector para almacenar los keypoints
  cv::cvtColor( image, image_gray, cv::COLOR_BGR2GRAY );//Se transforma la imagen a escala de grises
  // Cálculo de las features, el argumento true es para habilitar la supresión de no máximos y el tipo 
  //"TYPE_9_16" señala las dimensiones de la circunferencia para evaluar el algoritmo
  cv::FAST(image_gray,keypoints,thresh_fast,true,cv::FastFeatureDetector::TYPE_9_16);
  // Dibujo de los keypoints sobre la imagen. 
  cv::drawKeypoints(image, keypoints, image_fast, cv::Scalar(0,255,0));
   imshow( "FAST DETECTOR", image_fast );
}
void timerCallback(const ros::TimerEvent&)
  {  
    //Se realiza una lectura de la fuente seleccionada
    cap.read(image);
    HARRIS_DETECTOR( 0, 0 );
    FAST_DETECTOR();
    /*SIFT_DETECTOR();
    SURF_DETECTOR();*/
    // Se muestra la imagen capturada
    
    cv::imshow("ORIGINAL IMAGE", image);
    cv::waitKey(5);
  }
  
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cv_template_video");
  ros::NodeHandle nh;
  //image = cv::imread(r1,-1);
  //cap.open(0, cv::CAP_ANY);
  cap.open("/home/kiqmc10/imgtest/img_p/video2.m4v");
  //Se realiza una lectura de la fuente seleccionada
  cap.read(image);
  // Se configura el nombre de la ventana en donde se mostrará el video
  cv::namedWindow("ORIGINAL IMAGE", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("FAST DETECTOR", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("HARRIS DETECTOR", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("SIFT DETECTOR", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("SURF DETECTOR", cv::WINDOW_AUTOSIZE);
  //Poisicionamiento de las ventanas en la pantalla principal. 
  moveWindow("ORIGINAL IMAGE", 0,0);
  moveWindow("HARRIS DETECTOR", image.cols,0);
  moveWindow("FAST DETECTOR", 2*image.cols,0);
  moveWindow("SIFT DETECTOR", 3*image.cols,0);
  moveWindow("SURF DETECTOR", 4*image.cols,0);

  //createTrackbar( "Threshold: ", "corners_window", &thresh, max_thresh, HARRIS );
   if (!cap.isOpened())  
  {
      ROS_INFO("ERROR! Unable to read file");
      return 1;
  }
  ros::Timer timer =  nh.createTimer(ros::Duration(1.0/60.0),timerCallback);
  ros::spin();
}



