#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

  cv::VideoCapture cap;
  cv::Mat image1;
  cv::Mat image2;
  cv::Rect box;
  std::string ruta;
  
  int h = 0, w = 0;
  int weight = 0;
  cv::Point2i p;
  bool drawing_box = false, REG_SEL = false;
  
  void onTrackbarSlide( int pos, void *) 
	{
		 // Para este caso no se hace nada con el llamado de la función
	}
	void draw_box( cv::Mat& img, cv::Rect box ) 
	{
		cv::rectangle(img, box.tl(), box.br(), cv::Scalar(0x00,0x00,0xff));
	}	
	void my_mouse_callback(	int event, int x, int y, int flags, void* param) 
	{
		cv::Mat& image = *(cv::Mat*) param;
			
		switch( event ) 
		{
			case cv::EVENT_MOUSEMOVE: 
			{
				if( drawing_box ) 
				{
					box.width = x-box.x;
					box.height = y-box.y;
					if( box.width < 0 ) 
					{
						box.x += box.width;
						box.width *= -1;
					}
					if( box.height < 0 ) 
					{
						box.y += box.height;
						box.height *= -1;
					}
				}
			}
			break;
			case cv::EVENT_LBUTTONDOWN: 
			{
					drawing_box = true;
					box = cv::Rect( x, y, 0, 0 );
			}
			break;
			case cv::EVENT_LBUTTONUP: 
			{
					drawing_box = false;					
			}
			break;
		}
	}
	void my_mouse_callback_2(	int event, int x, int y, int flags, void* param) 
	{
		cv::Mat& image = *(cv::Mat*) param;
		if (event == cv::EVENT_LBUTTONDOWN)
		{
			REG_SEL = true;
			p.x = x;
			p.y = y;		
		}
	}
  
  void timerCallback(const ros::TimerEvent&){  
    
    cap.read(image1);
  	
  	
    if (image1.empty()) 
    {
      ROS_INFO("could not read image");
      return; 
    }
    
    if (REG_SEL)
    {
    	w = box.width;
    	h = box.height;
    	
    	if ((p.x + w)>image1.cols) 
    		w = image1.cols - p.x;   
    	if((box.x + w)>image2.cols)  
    		w = image2.cols - box.x; 	
    	if ((p.y + box.height)>image1.rows) 
    		h = image1.rows - p.y;   	
    	if ((box.y + h)>image2.rows)
    		h = image2.rows - box.y;
    		
    	if (box.x < 0) box.x = 0;
    	if (box.y < 0) box.y = 0;
    	
    	
		  cv::Mat roi1(image1, cv::Rect(p.x, p.y, w, h));
		  cv::Mat roi2(image2, cv::Rect(box.x, box.y, w, h));
		  cv::addWeighted(roi1,weight/100.0,roi2,1-weight/100.0,0.0,roi1); 
		}
    cv::imshow("Window", image1);
    cv::waitKey(10);
    
    if( drawing_box )
    {
    	image2 = cv::imread(ruta, cv::IMREAD_COLOR);
    	draw_box( image2, box );
    }
    cv::imshow("Window2", image2);
    cv::waitKey(10);
    
  }
  
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cv_template");
  ros::NodeHandle nh;
  cap.open(0, cv::CAP_ANY);
  ruta = argv[1];
  
  image2 = cv::imread(ruta, cv::IMREAD_COLOR);
  
  // Configuración de ventanas donde se mostrarán imágenes o videos
  cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Window2", cv::WINDOW_GUI_EXPANDED);
  
  // Configuración de acción al pasar mouse alguna de las ventanas anteriores
  cv::setMouseCallback(	"Window2",	my_mouse_callback,(void*)&image2);
  cv::setMouseCallback(	"Window",	my_mouse_callback_2,(void*)&image1);
  
	// Configuración de la Trackbar para hacer efecto blend 
  cv::createTrackbar("Blend", "Window", &weight, 100 ,onTrackbarSlide);
	// Mensaje informativo en caso de no detectar cámara
  if (!cap.isOpened())  
  {
      ROS_INFO("ERROR! Unable to open camera");
      return 1;
  }
  // Si la cámara funciona correctamente se imprime en ROS_INFO que se ha comenzado a grabar
  ROS_INFO("Start grabbing...");
  ros::Timer timer =    nh.createTimer(ros::Duration(1.0/24.0),timerCallback);
  ros::spin();
}


