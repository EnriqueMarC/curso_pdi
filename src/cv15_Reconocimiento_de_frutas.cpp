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
std::vector<std::vector<cv::Point2i>> contours;
cv::Mat image, image_, image_c;
std::vector<cv::Vec4i> hierarchy;
std::vector<cv::Mat> firmas;
Mat firma_comp;
cv::Mat image_f = cv::Mat::zeros(500,1000,CV_8UC3);
std::string r1 = "/home/kiqmc10/imgtest/img_p/platano2.jpg";
cv::Scalar colores[8] = {cv::Scalar(255,128,  0),cv::Scalar(  0,  0,255),cv::Scalar(  0,255,  0),cv::Scalar(  0,255,255),
                            cv::Scalar(255,  0,  0),cv::Scalar(255,  0,255),cv::Scalar(255,255,  0),cv::Scalar(255,255,255)};
Point centroide;
  
  void bordesCerrados()
  {
    /*En esta función se buscan los bordes cerrados detectados. Los que no cumplan con esto
    son eliminados de la variable contours y de hierarchy, se hace un decrementos en el contador
    debido a que al eliminar el borde se recorren los elementos y es necesario revisar esa ubicación de nuevo*/
    for (int c = 0; c < contours.size(); c++)
    {
        if (contourArea(contours[c])<1000){
        contours.erase(contours.begin()+c);
        hierarchy.erase(hierarchy.begin()+c);
        c--;
      }
    }
  }
	void calcular_firmas()
  {
    firmas.clear();
    cv::Mat fir = cv::Mat::zeros(360,1,CV_16UC1);
    //printf("Cantidad de contornos: %i \n", contours.size());
    for (int c = 0; c < contours.size(); c++)
    { 
      firmas.push_back(cv::Mat::zeros(360,1,CV_32SC1));
      cv::Moments M = cv::moments(contours[c]);
      cv::Point p;
      p.x = M.m10/M.m00;
      p.y = M.m01/M.m00;
      centroide = p;
      for (int e = 0; e<contours[c].size(); e++)
      {
        cv::Point2i dp = p - contours[c].at(e);        
        int ang = (int)(atan2(dp.y,dp.x)*180.0/PI);
        if (ang < 0) ang = 360 + ang;
        firmas[c].at<int>(ang,0) = (int) cv::norm(dp);
      }
      
    }      
  }
  void dibujarFirma(cv::Mat hist, cv::Mat hist_image, std::string window_name, cv::Scalar color)
  {
    //Normalización de los valores del array hist para estar dentro de los rangos
    cv::Mat hist_;
    cv::normalize(hist, hist_, hist_image.rows, 0,cv::NORM_INF);
    int hist_size = hist.checkVector(1);
    int bin_w = cvRound( (double) hist_image.cols/hist_size );
    // Creación del dibujo
    for( int i = 1; i < hist_size; i++ )
    {   // Se realiza la intepolación de los puntos de la variable hist y se almacena en hist_image
        cv::line( hist_image, cv::Point( bin_w*(i-1), hist_image.rows - cvRound(hist_.at<int>(i-1)) ),
                  cv::Point( bin_w*(i), hist_image.rows - cvRound(hist_.at<int>(i)) ),
                  color, 2, 8, 0  );
    }
    cv::imshow(window_name, image_f);
    cv::waitKey(5);
  }
  void dibujarFirmas(std::string window_name)
  {
    image_f = cv::Mat::zeros(500,1000,CV_8UC3);
    //printf("tamaño de firmas: %i \n",firmas.size());
    for (int e=0; e<firmas.size(); e++)
      {
        dibujarFirma(firmas[e], image_f,window_name,colores[e%8]);
      }
    
  }

  void rellenarFirmas()
  { 
    cv::Point p;
    // Encontrar espacios vacíos
    std::vector<cv::Point> espacios;
    cv::Mat firmas_r = cv::Mat::zeros(360,1,CV_32SC1);
    
    bool flag = false, flag_ = false;
    //std::cout << firmas[0] << std::endl;
    for (int e = 0; e < firmas.size(); e++) 
    {
      espacios.clear();
      for (int c = 0; c <360;c++)
      {
        if (firmas[e].at<int>(c,0) == 0)
        {
          if (flag_ == 1)
            espacios.push_back(Point(c-1,firmas[e].at<int>(c-1,0)));

          flag  = 1;
          flag_ = 0;
        }
        else
        {
          flag = 0;
        }

        if (flag == 0 && flag_ == 0)
        {
          espacios.push_back(Point(c,firmas[e].at<int>(c,0)));
          flag_ = 1;
        }
      }
      //cout << espacios << endl; 
      
      float xi, yi, xf, yf, dx;
      for (int c = 2; c < espacios.size(); c+=2)
      {
        xi = espacios[c-1].x;
        yi = espacios[c-1].y;
        xf = espacios[c].x;
        yf = espacios[c].y;
        
        for (int x = xi; x < xf; x++)
        {
          firmas[e].at<int>(x,0) = (int)(((yf-yi)/(xf-xi))*(x-xi)+yi);
        }
      }
      xi = espacios[espacios.size()-1].x;
      yi = espacios[espacios.size()-1].y;
      xf = espacios[0].x;
      yf = espacios[0].y;
      dx = 360 - xi + xf;

      for (int x = xi; x < 360; x++)
        {
          firmas[e].at<int>(x,0) = (int)(((yf-yi)/(dx))*(x-xi)+yi);
        }
      xi = 0;
      yi = firmas[e].at<int>(359,0);
      for (int x = 0; x < xf; x++)
      {
        firmas[e].at<int>(x,0) = ((yf-yi)/(dx))*(x-xi)+yi;
      }

      }
  }

  void guardarfirmas()
  {
    std::ofstream myfile;
    myfile.open("/home/kiqmc10/firma2.txt");
    for (int x = 0; x<360;x++)
      myfile << firmas[0].at<int>(x,0) << endl;
    myfile.close();

  }

  void cargar_firma()
  {
    ifstream myfile ("/home/kiqmc10/firma.txt");
    firma_comp = cv::Mat::zeros(360,1,CV_32SC1);
    string data;
    int c = 0;
    if (myfile.is_open())
    {
      while(getline(myfile,data))
      {
        firma_comp.at<int>(c,0) = stoi(data);
        c++;
      }
      myfile.close();
    }
    else
      cout << "archivo no disponible" << endl;
  }

  void analisis_firmas()
  {
    Mat error = cv::Mat::ones(360,1,CV_32SC1)*1000;
    vector<int> firma_c;
    uint error_min = -1;
    uint angulo = 0;
    for (int f = 0; f < firmas.size(); f++)
    {
      for (int e = 0; e < 360; e++)
      {
        error.at<int>(e,0) = norm(firmas[f],firma_comp);

        if (error.at<int>(e,0) < error_min)
        {
          error_min = error.at<int>(e,0);
          angulo = e;
        }

        firma_c.clear();
        int dato_1 = firmas[f].at<int>(0,0);

          for (int j = 1; j<360; j++)
          {
            firma_c.push_back(firmas[f].at<int>(j,0));
          }
          firma_c.push_back(dato_1);
          for (int j = 0; j<360; j++)
          {
            firmas[f].at<int>(j,0) = firma_c[j];
          }
      }

      if (error_min < 300)
      {
          cout << "Plátano detectado, está rotado "<< angulo <<"°" << " de la firma de muestra"<< endl;
          cout << "La firma "<< f + 1 << " es un Plátanos" << endl;
      }
      else
          cout << "Objeto no identificado" << endl;
    }
      

        
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

    // La función flip invierte la imagen con respecto a la vertical
    cv::flip(image, image,1);
    // Conversión de imagen a escala de grises
    cv::cvtColor(image,image_,cv::COLOR_BGR2GRAY);
    // Aplicación de filtro para eliminación de ruido
    cv::medianBlur(image_, image_, 7);
    // Detección de bordes de la imagen con el algoritmo de canny
    cv::Canny(image_,image_,50,200,3);
    // Función para detección de bordes
    findContours( image_, contours, hierarchy, RETR_EXTERNAL , CHAIN_APPROX_SIMPLE );
    // Clonación de imagen para dibujar sobre la original los bordes
    Mat drawing = image.clone();
    // Detección de bordes cerrados
    bordesCerrados();
    // Dibujo de bordes
    for( size_t i = 0; i< contours.size(); i++ )      
        cv::drawContours( drawing, contours, (int)i, colores[i%8], 3, cv::LINE_8);
    // cálculo de las firmas encontradas.
    calcular_firmas();
    // Relleno de los espacios donde es cero la distancia. Se utiliza interpolación lineal.
    rellenarFirmas();
    //Normalización de las firmas para que tengan un valor máximo de 100
    for(int f = 0; f < firmas.size(); f++)
      cv::normalize(firmas[f], firmas[f], 100, 0,cv::NORM_INF);   

    // --- Descomentar la siguiente línea para guardar la firma de la imagen de muestra
     // guardarfirmas();

    dibujarFirmas("Firmas2");

    cargar_firma();

    analisis_firmas();

    // Dibujo del centroide sobre la imagen para corroborar que está dentro del área de la imagen
    circle(drawing, centroide, 5, Scalar(255,0,0), cv::FILLED);

    
    // Se muestra la imagen capturada
    cv::imshow("Window", drawing);
    cv::imshow("Window_", image_);
    cv::waitKey(5);
  }
  
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cv_template_video");
  ros::NodeHandle nh;
  //image = cv::imread(r1,-1);
  cap.open(0, cv::CAP_ANY);
  
  // Se configura el nombre de la ventana en donde se mostrará el video
  cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Window_", cv::WINDOW_AUTOSIZE);
  //cv::namedWindow("Firmas", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Firmas2", cv::WINDOW_AUTOSIZE);
  moveWindow("Window", 0,0);
  moveWindow("Window_",0,540);
  //moveWindow("Firmas", 920,0);
  moveWindow("Firmas2", 920,540);
   if (!cap.isOpened())  
  {
      ROS_INFO("ERROR! Unable to open camera");
      return 1;
  }
  ros::Timer timer =  nh.createTimer(ros::Duration(1.0/60.0),timerCallback);
  ros::spin();
}



