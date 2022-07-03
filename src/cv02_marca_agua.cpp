#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

  cv::VideoCapture cap;
  cv::Mat image1;
  cv::Mat image2;
  using namespace cv;
  int weight = 0;
  int x = 0, y = 0, h = 0, w = 0;
  
   void onTrackbarSlide( int pos, void *) {
		 // Para este caso no se hace nada con el llamado de la función
	}
  
  void timerCallback(const ros::TimerEvent&){  
    
    cap.read(image1);
    w = image2.cols;
  	h = image2.rows;
  	
    if (image1.empty()) 
    {
      ROS_INFO("could not read image");
      return; 
    }
    //cv::Mat roi1(image1, cv::Rect(x, y, w, h));
    //cv::Mat roi2(image2, cv::Rect(x, y, w, h));
    //cv::addWeighted(roi1,weight/100.0,roi2,1-weight/100.0,0.0,roi1);    
    
    /*for (int y = 0; y < w; y++)
    {
    	for (int x = 0; x < h; x++)
		  {	
		  	cv::Vec3b& pixel1 = image1.at<cv::Vec3b>(x,y);	
		  	cv::Vec3b  pixel2 = image2.at<cv::Vec3b>(x,y);
		  	
		  	pixel1[0] = (int)(pixel1[0]*(float)weight/100.0 + pixel2[0]*(1.0 - (float)weight/100.0));
		  	pixel1[1] = (int)(pixel1[1]*(float)weight/100.0 + pixel2[1]*(1.0 - (float)weight/100.0));
		  	pixel1[2] = (int)(pixel1[2]*(float)weight/100.0 + pixel2[2]*(1.0 - (float)weight/100.0));
		  }
    }*/
    
    cv::Mat roi1(image1, cv::Rect(0, 0, w, h));
    cv::Mat roi2(image2, cv::Rect(0, 0, w, h));
   
		roi1 = roi1 * weight/100.0 +  roi2 * (1 - weight/100.0);
    
    cv::imshow("Window", image1);
    cv::waitKey(10);
  }
  
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cv_template");
  ros::NodeHandle nh;
  cap.open(0, cv::CAP_ANY);
  
  image2 = cv::imread(argv[1], cv::IMREAD_COLOR);
  
  
  // Se configura el nombre de la ventana en donde se mostrará el video
  cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);
	
	// Si no se detecta la cámara el nodo retorna un 1 y finaliza. Escribe en la terminal que no fue posible abrir la cámara.
  if (!cap.isOpened())  
  {
      ROS_INFO("ERROR! Unable to open camera");
      return 1;
  }
  
  
  cv::createTrackbar("Blande", "Window", &weight, 100 ,onTrackbarSlide);
  // Si la cámara funciona correctamente se imprime en ROS_INFO que se ha comenzado a grabar
  ROS_INFO("Start grabbing...");
  ros::Timer timer =    nh.createTimer(ros::Duration(1.0/24.0),timerCallback);
  ros::spin();
}


