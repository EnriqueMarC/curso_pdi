#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

cv::VideoCapture cap;
cv::Mat frame;
sensor_msgs::ImagePtr msg;

image_transport::Publisher pub ;

void timerCallback(const ros::TimerEvent&)
{  
  	cap >> frame; 
    if(!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      //cv::imshow("Window", frame);
      cv::waitKey(5);
    }

    ros::spinOnce();  
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cv_template_video");
  ros::NodeHandle nh; 
  image_transport::ImageTransport it(nh);
  //pub = it.advertise("camera/image_raw", 1);
  pub = it.advertise("/image_cam_rpi", 1);
  ROS_INFO("Start cv_template_video"); 
  
  //cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);
 
  cap.open(1, cv::CAP_ANY);
  //cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
  //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);



  if (!cap.isOpened())  
  {
      ROS_INFO("ERROR! Unable to open camera");
      return 1;
  }
  cap >> frame;

  ROS_INFO("Image Resolution: %d X %d",frame.cols, frame.rows );
  ROS_INFO("Start grabbing...");
  ros::Timer timer = nh.createTimer(ros::Duration(1.0/24.0), timerCallback); 
  ros::spin();
}