#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
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
int thresh = 100;
int max_thresh = 255;
std::string r1 = "/home/kiqmc10/imgtest/img_p/platano2.jpg";
void cornerHarris_demo( int, void* )
{
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;
    Mat dst = Mat::zeros( image.size(), CV_32FC1 );
    cornerHarris( image_gray, dst, blockSize, apertureSize, k );
    Mat dst_norm, dst_norm_scaled;
    normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    convertScaleAbs( dst_norm, dst_norm_scaled );
    for( int i = 0; i < dst_norm.rows ; i++ )
    {
        for( int j = 0; j < dst_norm.cols; j++ )
        {
            if( (int) dst_norm.at<float>(i,j) > thresh )
            {
                circle( dst_norm_scaled, Point(j,i), 5,  Scalar(0), 2, 8, 0 );
            }
        }
    }
    imshow( "corners_window", dst_norm_scaled );
}
void timerCallback(const ros::TimerEvent&)
  {  
    //Se realiza una lectura de la fuente seleccionada
    cap.read(image);

    //image = cv::imread(r1,-1);

    if (image.empty())
    {
      ROS_INFO("could not read image");
      return;
    }
    cvtColor( image, image_gray, COLOR_BGR2GRAY );
    createTrackbar( "Threshold: ", "corners_window", &thresh, max_thresh, cornerHarris_demo );
    cornerHarris_demo( 0, 0 );

   
    // Se muestra la imagen capturada
    cv::imshow("Window", image);
    cv::waitKey(5);
  }
  
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cv_template_video");
  ros::NodeHandle nh;
  //image = cv::imread(r1,-1);
  cap.open(0, cv::CAP_ANY);
  //Se realiza una lectura de la fuente seleccionada
  cap.read(image);
  // Se configura el nombre de la ventana en donde se mostrará el video
  cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("corners_window", cv::WINDOW_AUTOSIZE);
  moveWindow("Window", 0,0);
  moveWindow("corners_window", image.cols,0);
  
   if (!cap.isOpened())  
  {
      ROS_INFO("ERROR! Unable to open camera");
      return 1;
  }
  ros::Timer timer =  nh.createTimer(ros::Duration(1.0/60.0),timerCallback);
  ros::spin();
}



