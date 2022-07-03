#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <cmath>
#include <random>
//---------------Declaración de variables----------------//
const int n_cat = 3; 
cv::Mat image , k_means_image, image_c, image_binary;
cv::Point2i p_ref;
cv::Scalar categoria[n_cat];
cv::VideoCapture cap;
std::string r1 = "/home/kiqmc10/imgtest/parrot2.jpg";
uchar e_c = 0;
unsigned int e_in = 0;
float error[n_cat];
int i_max = 5;
unsigned long int contador[n_cat];
int cat_s;
bool ready = false;
// Código para generar número aleatorios uniformemente distribuidos
std::random_device dev;
std::mt19937 rng(dev());
std::uniform_int_distribution<std::mt19937::result_type> dist6(0,255); 

	
void timerCallback(const ros::TimerEvent&){ 

  ready = true;
  image_binary = cv::Mat::zeros(image.rows, image.cols, CV_8UC1); // Reset de valores de imagen binaria
  /*Activación de algoritmo por bandera. Cambia estado a True una vez seleccionado un 
    pixel en la imagen de la ventana Window*/
  if (ready){
  for (int i = 0; i < i_max; i++) // Ciclo que determina las veces que se ejecuta el algoritmo k-means
  {
    for (int x = 0; x < image.cols; x++) // Ciclos para hacer el recorrido en cada pixel de la imagen 
    {
      for (int y = 0; y < image.rows; y++)
      {
        cv::Vec3b pixel = image.at<cv::Vec3b>(y, x);// Alamacenamiento de componentes RGB del pixel x,y

        e_in = pow(255,2)*n_cat + 1; // Se calcula el máximo error posible para comparar los errores obtenidos en error[c]

        for (int c = 0; c < n_cat; c++)
        { // cálculo del error de la intensidad del pixel x,y respecto de cada categoría.
          error[c] = pow(abs(pixel.val[0]- categoria[c][0]),2) +
                     pow(abs(pixel.val[1]- categoria[c][1]),2) + 
                     pow(abs(pixel.val[2]- categoria[c][2]),2) ;

          if (error[c] < e_in) // Selección del error más pequeño
          {
            e_in = error[c]; // Actualización de la variable e_in para comparar lo errores siguientes.
            e_c = c; // selección de la categoría más cercana al pixel x,y
          }
        }
          image_c.at<uchar>(y,x) = e_c; // Almacenamiento de la categoría a la que pertenece cada pixel.
      }
    }
  for (int c = 0; c < n_cat; c++)
  {   // Limpieza de las categorías para hacer el cálculo de las nuevas
      categoria[c] = cv::Scalar(0,0,0);
      contador[c]  = 0;
  }

  for (int x = 0; x < image.cols; x++)
  {
    for (int y = 0; y < image.rows; y++)
    {
        cv::Vec3b& pixel_p = image.at<cv::Vec3b>(y, x);
        uchar& cat = image_c.at<uchar>(y,x);
        contador[cat]++;

        if (x == p_ref.x && y == p_ref.y && i == i_max - 1) 
          { // Se almacena la categoría a la que pertenece el pixel seleccionado al inicio del código.
            cat_s = cat;
          }
          // Cálculo del promedio de forma recursiva
        categoria[cat][0] = ((contador[cat]-1)*categoria[cat][0] + pixel_p.val[0])/contador[cat];
        categoria[cat][1] = ((contador[cat]-1)*categoria[cat][1] + pixel_p.val[1])/contador[cat];
        categoria[cat][2] = ((contador[cat]-1)*categoria[cat][2] + pixel_p.val[2])/contador[cat];         
    }
  }
}
// Creación de la imagen segmentada y de la imagen binaria.
for (int x = 0; x < image.cols; x++)
  {
    for (int y = 0; y < image.rows; y++)
    { // ------ Generación de imagen segmentada ----------
      cv::Vec3b& pixel_k = k_means_image.at<cv::Vec3b>(y,x);
      uchar& e_c = image_c.at<uchar>(y,x);
      pixel_k.val[0] = categoria[e_c][0];
      pixel_k.val[1] = categoria[e_c][1];
      pixel_k.val[2] = categoria[e_c][2];
      // ------ Generación de imagen binaria -------
      if (e_c == cat_s)
        image_binary.at<uchar>(y,x) = 255;
    }
  }
  ready = false;
}
  // Impresión de imágenes: Segmentada y binaria
  cv::imshow("Window", image);
  cv::imshow("Window_", k_means_image);
  cv::imshow("Window_2", image_binary);
  cv::waitKey(5);
  }
void my_mouse_callback( int event, int x, int y, int flags, void* param) 
  {
      if (event == cv::EVENT_LBUTTONUP)
      { 
        p_ref = cv::Point2i(x,y); // Obtención de coordenadas del punto deseado
        ready = true; // Activación de bandera para inicio de algoritmo k-means
      }
  }

int main(int argc, char** argv)
{ 
  ros::init(argc, argv, "cv_09_k_means");
  ros::NodeHandle nh;

  // Lectura de la imagen de prueba
  image = cv::imread(r1,-1);
  // Inicialización de imagen binaria
  image_binary = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
  if (image.empty())
    {
      ROS_INFO("could not read image");
      return 1;
    } 
  // Creación de ventanas para mostrar las tres imágenes.
  cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Window_", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Window_2", cv::WINDOW_AUTOSIZE);
  // Creación del callback del mouse
  cv::setMouseCallback( "Window",  my_mouse_callback,(void*)&image);
  // Se muestra imagen de prueba
  cv::imshow("Window", image);
  cv::waitKey(33);
  // Creación de categorías de forma aleatoria, se crean n_cat categorías
  for (int i = 0; i < n_cat; i++)
   {
     categoria[i] = cv::Scalar(dist6(rng),dist6(rng),dist6(rng));
   } 
   // Inicialización de imagen que muestra la imagen segmentada
  k_means_image = cv::Mat::zeros(image.rows,image.cols,CV_8UC3); 
   // Inicialización de matriz que almacena a que categoría pertenece cada pixel.
  image_c = cv::Mat::zeros(image.rows,image.cols,CV_8UC1);
   // Configuración del Temporizador a 24 FPS. 
  ros::Timer timer = nh.createTimer(ros::Duration(1.0/24.0),timerCallback);
  ros::spin();
}



