#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// Declaración de variables para la ejecución del programa
cv::VideoCapture cap;
cv::Mat image, image_;
cv::Mat R, G, B;
// Rutas donde se encuentran las imágenes de prueba
std::string r1 = "/home/kiqmc10/imgtest/Prueba1.jpg";
std::string r2 = "/home/kiqmc10/imgtest/Prueba2.jpg";
std::string r3 = "/home/kiqmc10/imgtest/Prueba3.jpg";
std::string r4 = "/home/kiqmc10/imgtest/dog.jpg";
std::string selector;

int alpha = 100, beta = 10;

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
    //Normalización de los valores del array hist para estar dentro de los rangos de la ventana
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
    image_ = image * beta/10.0 + (alpha - 100); 
    
    // Creación de variable hist que almacenará el arreglo con los valores del histograma
    cv::Mat hist;
    cv::Mat hist_image(600, 800, CV_8UC3, cv::Scalar(0,0,0) );    
    
    simpleHistogram(&image_, hist, 256);
    drawHistogram(hist, hist_image, "Window_hist",cv::Scalar(0, 0, 255));   
    cv::waitKey(1);
    // Impresión de la imagen original en la ventana Window_image
    cv::imshow("Window_image", image_);
    cv::waitKey(5);
}

void trackbarCallback( int pos, void *) 
  {
     // Para este caso no se hace nada con el llamado de la función
  }

int main(int argc, char** argv)
{
  //Inicialización del nodo de ROS
  ros::init(argc, argv, "cv06_brillo_contraste");
  ros::NodeHandle nh; 
  ROS_INFO("Start cv06_brillo_contraste"); 
  // Lectura del argumento 1 que permite seleccionar si entra la cámara y las imágenes
  selector = argv[1];
  // Inicialización de las ventanas en las que se muestra la imagen fuente y el histograma

  cv::namedWindow("Window_image", cv::WINDOW_AUTOSIZE);
  //cv::namedWindow("Window", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Window_hist", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Window_",cv::WINDOW_AUTOSIZE);
  // Selección de la fuente de la imagen
  if (selector == "0") image = cv::imread(r1, cv::IMREAD_GRAYSCALE);
  else if (selector == "1") image = cv::imread(r1, cv::IMREAD_GRAYSCALE);
  else if (selector == "2") image = cv::imread(r2, cv::IMREAD_GRAYSCALE);
  else if (selector == "3") image = cv::imread(r3, cv::IMREAD_GRAYSCALE);
  else image = cv::imread(r4, cv::IMREAD_GRAYSCALE);
 
  cv::imshow("Window_",image);
  cv::waitKey(33);
  ROS_INFO("Read image");



  cv::createTrackbar("alpha","Window_image",&alpha,200,trackbarCallback);
  cv::createTrackbar("beta", "Window_image", &beta, 20,trackbarCallback);

  ros::Timer timer = nh.createTimer(ros::Duration(1.0/24.0), timerCallback); 
  ros::spin();
}