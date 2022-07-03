#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// Declaración de variables para la ejecución del programa
cv::VideoCapture cap;
cv::Mat image;
cv::Mat R, G, B;
// Rutas donde se encuentran las imágenes de prueba
std::string r1 = "/home/kiqmc10/imgtest/RED.png";
std::string r2 = "/home/kiqmc10/imgtest/GREEN.png";
std::string r3 = "/home/kiqmc10/imgtest/BLUE.png";
std::string selector;

// Función para calcular el histograma
void simpleHistogram(const cv::Mat* src, cv::OutputArray hist, float maxValue)
{
  int hist_size = maxValue;
  float range[] = { 0, maxValue };
  const float* hist_range[] = { range };
  // Función calcHist hace el cómputo de intensidades de los pixeles y lo devuelve en la variable hist
  cv::calcHist( src, 1, 0, cv::Mat(), hist, 1, &hist_size, hist_range);
}

void drawHistogram(cv::Mat hist, cv::Mat hist_image, std::string window_name, cv::Scalar color)
{
    //Normalización de los valores del array hist para estar dentro de los rangos
    cv::normalize(hist, hist, 0, hist_image.rows, cv::NORM_MINMAX, -1, cv::Mat() );
    int hist_size = hist.checkVector(1);
    int bin_w = cvRound( (double) hist_image.cols/hist_size );
    // Creación del dibujo
    for( int i = 1; i < hist_size; i++ )
    {   // Se realiza la intepolación de los puntos de la variable hist y se almacena en hist_image
        cv::line( hist_image, cv::Point( bin_w*(i-1), hist_image.rows - cvRound(hist.at<float>(i-1)) ),
                  cv::Point( bin_w*(i), hist_image.rows - cvRound(hist.at<float>(i)) ),
                  color, 2, 8, 0  );
    }
    // Se muestra dibujo en la ventana seleccionada
    cv::imshow(window_name, hist_image);
}

void timerCallback(const ros::TimerEvent&)
{  
    // Selector que permite escoger la fuente de la imagen
    if (selector == "0") 
        cap >> image; //cap.read(image);
    else if (selector == "1") 
        image = cv::imread(r1, -1);
    else if (selector == "2") 
        image = cv::imread(r2, -1);
    else 
        image = cv::imread(r3, -1);

    if (image.empty()) 
    {
      ROS_INFO("could not read image");
      return; 
    }
    // Creación de variable hist que almacenará el arreglo con los valores del histograma
    cv::Mat hist;
    cv::Mat hist_image(600, 800, CV_8UC3, cv::Scalar(0,0,0) );
    
    // Extracción de canales RGB de la imagen o video. La matriz con los valores 
    // se almacena en las matrices R, G, B de tipo cv::Mat   
    cv::extractChannel(image,R,0);
    cv::extractChannel(image,G,1);
    cv::extractChannel(image,B,2);
    // Creación del histograma para el canal 0 correspondiente al color rojo
    simpleHistogram(&R, hist, 256);
    drawHistogram(hist, hist_image, "Window_hist",cv::Scalar(255, 0, 0));   
    // Creación del histograma para el canal 1 correspondiente al color verde
    simpleHistogram(&G, hist, 256);
    drawHistogram(hist, hist_image, "Window_hist",cv::Scalar(0, 255, 0));   
    // Creación del histograma para el canal 2 correspondiente al color azul
    simpleHistogram(&B, hist, 256);
    drawHistogram(hist, hist_image, "Window_hist",cv::Scalar(0, 0, 255));   
    cv::waitKey(1);
    // Impresión de la imagen original en la ventana Window_image
    cv::imshow("Window_image", image);
    cv::waitKey(5);
}


int main(int argc, char** argv)
{
  //Inicialización del nodo de ROS
  ros::init(argc, argv, "cv04_histograma");
  ros::NodeHandle nh; 
  ROS_INFO("Start cv04_histograma"); 
  // Lectura del argumento 1 que permite seleccionar si entra la cámara y las imágenes
  selector = argv[1];
  // Inicialización de las ventanas en las que se muestra la imagen fuente y el histograma
  cv::namedWindow("Window_image", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Window_hist", cv::WINDOW_AUTOSIZE);

  // Configuración para seleccionar la cámara de la laptop como fuente de video SSI selector == 0
  if (selector == "0")
  {
  cap.open(0, cv::CAP_ANY);

  if (!cap.isOpened())  
  {
      ROS_INFO("ERROR! Unable to open camera");
      return 1;
  }
}
  //ROS_INFO("Start grabbing...");
  ros::Timer timer = nh.createTimer(ros::Duration(1.0/24.0), timerCallback); 
  ros::spin();
}