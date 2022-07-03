#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
// ----------- Declaración de Variables ----------------//
cv::Mat image, image_, image_p, image_v;
cv::Vec3b referencia, prom ;
cv::Point2i p_ref;
cv::VideoCapture cap;
std::string r1 = "/home/kiqmc10/imgtest/parrot2.jpg";
bool ready = false;
int tolerancia = 50;
const int num_vec = 4;
int dist[num_vec];
int sel = 0;


void timerCallback(const ros::TimerEvent&)
{ 
  prom = image.at<cv::Vec3b>(p_ref.y, p_ref.x);
  // Si la bandera está en true, el algoritmo comienza a procesar la información
  //ready = true;
  image_ = cv::Mat::zeros(image.rows, image.cols, CV_8UC1); // Reset de matriz image_ 
  while (ready)
  {

    // Variable para generar el paro del algoritmo cuando se haya culminado el crecimiento de la región
    ready = false;
    // Reset de matriz para almacenar datos nuevos
    image_p = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
    image_.at<uchar>(p_ref.y,p_ref.x) = 255; // inicialización de matriz image_ con el punto semilla
    /*Comienzo de la evaluación de la intensidad de cada pixel y si este es vecino de algún pixel seleccionado*/
    for(int x = 1; x < image.cols - 1 ;x++)
    {
      for (int y = 1; y < image.rows - 1; y++)
      { // Se almacena la intensidad de las componentes RGB del pixel x,y
        cv::Vec3b pixel = image.at<cv::Vec3b>(y,x);
        /* Si en la posición x,y de la matriz image_ se almacenó 0, se evalúa si algún pixel vecino ha sido seleccionado, 
            si existe y además el pixel x,y cumple la condición de que la norma de su intensidad respecto del promedio
            es menor a la tolerancia, se almacena en la posición x,y de la matris image_p* un 255, en caso de no cumplirse
            se deja el valor de cero */
        
        if (image_.at<uchar>(y,x) == 0) 
        {     /*En la siguiente condición se evalúa si algún pixel es vecino del pixel x,y*/

          if (image_.at<uchar>(y-1,x) == 255 || image_.at<uchar>(y,x-1) == 255 || 
              image_.at<uchar>(y+1,x) == 255 || image_.at<uchar>(y,x+1) == 255 ||
              image_.at<uchar>(y-1,x-1) == 255 || image_.at<uchar>(y+1,x-1) == 255 || 
              image_.at<uchar>(y+1,x+1) == 255 || image_.at<uchar>(y-1,x+1) == 255)
          {
              /*En caso afirmativo, se calcula la norma entre el valor de intensidad del pixel x,y y el promedio
                si resulta ser menor a la tolerancia, se actualiza el valor de image_p en la posición x,y con 255*/              
            if ((cv::norm(pixel,prom)*255.0)/(sqrt(3*pow(255,2)))<tolerancia)
            {
              image_p.at<uchar>(y,x) = 255;
              // Si el algoritmo entró en esta sección se actualiza el valor de ready a true, para no detener la siguiente iteración
              ready = true; 
            }
          }
        }
      }
    }
    // Actualización de la matriz image_ con los nuevos pixeles localizados por el algoritmo
    image_ = image_ + image_p;
    //Reseteo de  valor de los promedios encontrados anteriormente de las intensidades de los pixeles
    /*prom = cv::Vec3b(0,0,0);
    int contador = 0;
    // Cálculo del nuevo promedio dados los nuevos valores de los pixeles seleccionados.
    for (int y = 0; y < image.rows; y++)
    {
      for (int x = 0; x < image.cols; x++)
      {
        if (image_.at<uchar>(y, x) == 255)
        { 
          cv::Vec3b pixel_p = image.at<cv::Vec3b>(y, x);
          contador++;
          prom.val[0] = ((contador-1)*prom.val[0] + pixel_p.val[0])/contador;
          prom.val[1] = ((contador-1)*prom.val[1] + pixel_p.val[1])/contador;
          prom.val[2] = ((contador-1)*prom.val[2] + pixel_p.val[2])/contador;
        }
      }
    }*/
    cv::imshow("Window_", image_);
  } 
  //Impresión de imagen procesada con el algoritmo Region Growing
  
  cv::imshow("Window", image);
  cv::waitKey(3);
}

void onTrackbarSlide( int pos, void *) {	}

void my_mouse_callback( int event, int x, int y, int flags, void* param) 
{
  if (event == cv::EVENT_LBUTTONUP)
  {   
    referencia = image.at<cv::Vec3b>(y,x); // Selección del pixel semilla. Almacenamiento de sus intensidades
    p_ref = {x,y}; // Almacenamiento de las coordenadas del punto semilla
    image_ = cv::Mat::zeros(image.rows, image.cols, CV_8UC1); // Reset de matriz image_ 
    image_.at<uchar>(p_ref.y,p_ref.x) = 255; // inicialización de matriz image_ con el punto semilla
    prom = referencia; //Inicialización del promedio con el valor de referencia
    ready = true; // Activación del algoritmo Region Growing con la variable ready
  }
}
int main(int argc, char** argv)
{
  //Inicialización del nodo
  ros::init(argc, argv, "cv_region_growing");
  ros::NodeHandle nh;
  image = cv::imread(r1, cv::IMREAD_COLOR);
    
  // Configuración de las ventanas que serán utilizadas para mostrar la imágenes
  cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);	 
  cv::namedWindow("Window_", cv::WINDOW_AUTOSIZE); 
  cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);  
  //Configuración de la trackbar para modificar la tolerancia
  cv::createTrackbar("Tol", "Window_", &tolerancia, 100 ,onTrackbarSlide);
  // Configuración del mousecallback. Se usa en este caso para inicializar el punto semilla
  cv::setMouseCallback( "Window",  my_mouse_callback,(void*)&image);
  //Lectura de la imagen de prueba

  //Inicialización de la matriz image_, en esta se almacenará la región de crecimiento. 
  image_ = cv::Mat::zeros(image.rows, image.cols, CV_8U);

  if (image.empty()) 
    {
      ROS_INFO("could not read image");
      return 1;
    }

  // Configuración del Timer para mostrar la imagen a 24 FPS
  ros::Timer timer = nh.createTimer(ros::Duration(1.0/24.0),timerCallback);  
  ros::spin();
}


