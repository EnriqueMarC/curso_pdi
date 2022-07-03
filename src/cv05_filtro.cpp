#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
cv::VideoCapture cap;
cv::Mat image;
/*float d_kernel[] = {1, 4, 7, 4,1,
                    4,16,26,16,4,
                    7,26,41,26,7,
                    4,16,26,16,4,
                    1, 4, 7, 4,1};*/
float d_kernel[] = {1,1,1,1,1,
                    1,1,1,1,1,
                    1,1,1,1,1,
                    1,1,1,1,1,
                    1,1,1,1,1};
cv::Mat kernel(5,5,CV_32FC1,d_kernel);
  

void timerCallback(const ros::TimerEvent&){  
  
  cap.read(image);
	cv::Mat image_(image.rows-5,image.cols-5, image.type(),cv::Scalar(0));  	
	
  if (image.empty()) 
  {
    ROS_INFO("could not read image");
    return; 
  }
  
 for (int x = 0; x < image.rows - 5; x++)
  {
  	for(int y = 0; y < image.cols - 5 ; y++)
  	{
  		for(int i = 0; i <= kernel.rows; i++)
  		{
  			for(int j = 0; j <= kernel.cols; j++)
  			{    			
  				image_.at<cv::Vec3b>(x,y) +=  (kernel.at<float>(i,j) * image.at<cv::Vec3b>(x+i,y+j));
  			}
  		}
  	}
  }
  //std::cout << "process ..." << image_.at<cv::Vec3b>(image.cols/2,image.rows/2)<<std::endl;
  //std::cout << "process ..." << kernel.at<float>(3,3) <<std::endl;
  
  cv::imshow("Window", image_);
  cv::waitKey(1);
  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cv_template");
  ros::NodeHandle nh;
  cap.open(0, cv::CAP_ANY);

  kernel = kernel * 1.0/25.0;
  // Se configura el nombre de la ventana en donde se mostrará el video
  cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);
  

  // Si no se detecta la cámara el nodo retorna un 1 y finaliza. Escribe en la terminal que no fue posible abrir la cámara.
  if (!cap.isOpened())  
  {
      ROS_INFO("ERROR! Unable to open camera");
      return 1;
  }

  // Si la cámara funciona correctamente se imprime en ROS_INFO que se ha comenzado a grabar
  ROS_INFO("Start grabbing...");
  ros::Timer timer =    nh.createTimer(ros::Duration(1.0/1.0),timerCallback);
  ros::spin();
}


