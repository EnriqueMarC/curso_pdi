#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp> // Librería de OpenCV
  
  // Creación de variable de tipo cv::Mat, esta clase permite almacenar
  // la información de la imagen leída. 
  cv::Mat image;
  
  //Función de interrupción. Se llama cada 41.66 ms para tener 24 FPS
  void timerCallback(const ros::TimerEvent&){  
    
    /* Ruta donde se ubica la imagen. El "1" es el formato en el que se
    mostrará la imagen, para este caso es cv::IMREAD_COLOR*/
    image = cv::imread("/home/kiqmc10/imgtest/Family.jpg", 1);
  	
    if (image.empty()) 
    {
    	/*Si no se localiza el archivo o no se puede leer, se imprime el mensaje 
    		siguiente:*/
      ROS_INFO("could not read image");
      return; 
    }
    /* La función imshow, muestra la imagen en la ventana ""Window*/
    cv::imshow("Window", image);
    /*Se necesita de un delay para que se pueda construir la imagen, la función
    cv::waitKey se puede usar también como una forma de limitar los FPS en caso
    de mostrar un video*/
    cv::waitKey(33);
    }
  
int main(int argc, char** argv)
{
	// Inicialización del nodo de ROS
  ros::init(argc, argv, "cv_template_file");
  ros::NodeHandle nh;
  ROS_INFO("Start cv_template_file");
  
  // Se configura el nombre de la ventana en donde se mostrará la imagen
  cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);	
	// Se crea la función de interrución, cada 41.66 ms o 24 FPS
  ros::Timer timer =  nh.createTimer(ros::Duration(1.0/24.0),timerCallback);
  ros::spin();
}


