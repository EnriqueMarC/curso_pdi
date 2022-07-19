#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d.hpp>
#include <math.h>
#include <iostream>
#include <fstream>
#include <iostream>
#include <fstream>

cv::VideoCapture cap;
cv::Mat image,image_c,image_p,image_f,difference,image_s,optical_flow;
cv::Vec3b pixel, pixel_f;
float alpha = 0.7;

// Función para general erosión en imágenes binarias
void myErode(cv::Mat img, int size)
{             
    cv::Mat element = cv::getStructuringElement( 0,cv::Size( 2*size + 1, 2*size+1 ),cv::Point( size, size ) );    
    cv::erode(img,img,element);
}
// Función para Generar dilatación en imágenes binarias.
void myDilate(cv::Mat img, int size)
{             
    cv::Mat element = cv::getStructuringElement( 0, cv::Size( 2*size + 1, 2*size+1 ), cv::Point( size, size ) );    
    cv::dilate(img,img,element);
}
// Función para calcular el flujo óptico denso.
void dense_optical_flow(cv::Mat& image_dof,cv::Mat image_p_dof, cv::Mat image_c_dof, cv::Mat image_s_dof)
{
    // Con la siguientes iteraciones se eliminan de la variable image_c_dof los pixeles que en la posicón x,y, están en cero en la
    //imagen binaria image_s_dof. Con esto se elimina el fondo,o los objetos estáticos en la escena. 
    for(int y = 0; y<image_dof.rows; y++)
        for(int x = 0; x<image_dof.cols; x++)
	        if(image_s_dof.at<uchar>( y, x ) == 0)
                image_c_dof.at<uchar>( y, x ) = 0;
  // La función siguiente permite calcular el flujo óptico denso usando el algoritmo de Gunnar Farneback, se compara una escena
  // previa con la actual. La aproximación se hace utilizando polinomios cuadráticos. 
    cv::calcOpticalFlowFarneback(image_p_dof, image_c_dof, optical_flow, 0.25, 2, 20, 3, 7, 1.5, 0);           
    // Las siguientes iteraciones permiten dibujar los vectores que describen el movimeinto. 
    for (int row = 4; row < image_dof.rows; row += 8)
        for(int column = 4; column < image_dof.cols; column += 8)
        {   // con las imágenes mostrando solo los objetos en movimiento, el cálculo del flujo óptico denso se aplica solo en dichas zonas
            cv::Point2f& flow = optical_flow.at<cv::Point2f>(row,column);
            if(cv::norm(flow)>2)
                cv::line (image_dof, cv::Point(column,row), cv::Point(cvRound(column+flow.x), cvRound(row+flow.y)), cv::Scalar(0,0,255));
        }     
}
// Función para calcular el flujo óptico basado en features. 
void feature_optical_flow(cv::Mat& image_fof,cv::Mat image_p_fof, cv::Mat image_c_fof, cv::Mat image_s_fof)
{   // Creación de variable que almacenará las características detectadas. 
    std::vector<cv::Point2f> features_p, features_c;
    //Cómputo de las carácterísticas usando el método de detección de esquinas por valor mínimo de eigenvalores
    cv::goodFeaturesToTrack(image_p_fof, features_p, 200, 0.01, 10, image_s_fof, 15, false, 0.04);
    //Cuando no existe movimeinto la cantidad de features es cero, por lo que en este caso no se realiza el cálculo de los vectores
    // en caso contrario se hace el cómputo.
    if((int)features_p.size()>0)
    {   // La siguiente función es encagarda de refinar la localización de las esquinas. 
        cv::cornerSubPix (image_p_fof, features_p, cv::Size(10, 10), cv::Size(-1,-1), cv::TermCriteria (cv::TermCriteria::EPS ,20,0.03));
        // Variable para almacenar caracteríisticas
        std::vector<uchar> features_f;
        // Cálculo de flujo óptico basado en características usando el método de Lucas-Kanade con pirámides
        cv::calcOpticalFlowPyrLK (image_p_fof, image_c_fof, features_p, features_c, features_f, cv::noArray());
        for( int i = 0; i < (int)features_p.size(); i++ )
            if(features_f[i])// Dibujo de los vectores sobre la imagen de salida.
                cv::line (image_fof, features_p[i], features_c[i], cv::Scalar(0,0,255));          
    }
}

void timerCallback(const ros::TimerEvent&)
{  
    // Lectura de la imagen de entrada
  	cap >> image; 
    // Reescalamiento de la imagen
  	cv::resize(image, image, cv::Size(640, 360), cv::INTER_LINEAR);
    // Aplicación del filtro pasa-bajas a la escena
    image_f = (1-alpha)*image + alpha*image_f; 
    // Cálculo de la diferencia absoluta entre la imagen actual y la imagen filtrada.
    cv::absdiff( image, image_f, difference ); 
    
  	image_s = cv::Mat::zeros( image.rows, image.cols, CV_8U);
  	// Creación de imagen binaria, por medio de la umbralización de la imagen resultante del cálculo de la diferenica absoluta anterior. 
    for(int y = 0; y<image.rows; y++)
        for(int x = 0; x<image.cols; x++)
	        if(norm(difference.at<cv::Vec3b>( y, x ))>50)
                image_s.at<uchar>( y, x ) = 255;
    // Aplicación de Erosión y dilatación a la imagen binaria para eliminación de ruido
    myErode(image_s, 3);    
    myDilate(image_s, 4);
    //cv::imshow("filter", image_s);
    //cv::waitKey(5);
    // Clonación de la imagen actual.
    image_c = image.clone();               
    cv::cvtColor(image_c,image_c,cv::COLOR_BGR2GRAY);
    cv::Mat image_dof = image.clone();
    cv::Mat image_fof = image.clone();
    //Cómputo del flujo ótico denso
    dense_optical_flow(image_dof, image_p, image_c, image_s);
    //Cómputo del flujo óptico basado en features
    feature_optical_flow(image_fof,image_p, image_c, image_s);
    // Impresión de las imágenes resultantes
    cv::imshow("Dense optical flow", image_dof);
    cv::imshow("Feature optical flow", image_fof);
    cv::waitKey(5);   
    // Actualización de la imagen previa. 
    image_p = image_c.clone();  
}

int main(int argc, char** argv)
{
  // Inicialización del nodo
  ros::init(argc, argv, "flujo_optico");
  ros::NodeHandle nh; 
  ROS_INFO("flujo_optico"); 
  // Configuración de la fuente de video y el tipo de imagen.
  cap.open(2, cv::CAP_ANY);

  if (!cap.isOpened())  
  {
      ROS_INFO("ERROR! Unable to open camera");
      return 1;
  }  
  // Lectura de la primer imagen
  cap >> image_f; 
  cv::resize(image_f, image_f, cv::Size(640, 360), cv::INTER_LINEAR);
  image_p = image_f.clone(); 
  cv::cvtColor(image_p,image_p,cv::COLOR_BGR2GRAY);
  // Inicialización de las ventanas
  cv::namedWindow("Dense optical flow", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Feature optical flow", cv::WINDOW_AUTOSIZE);
  cv::moveWindow("Dense optical flow", 0,0);
  cv::moveWindow("Feature optical flow", image_f.cols,0);
  // Configuración del callback
  ros::Timer timer = nh.createTimer(ros::Duration(1.0/40.0), timerCallback); 
  ros::spin();
}