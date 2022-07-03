cmake_minimum_required(VERSION 3.0.2)
project(curso_pdi)

set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  OpenCV
  message_generation
  geometry_msgs
)

add_message_files(
	FILES
	VectorAruco.msg
	aruco.msg
 )
 generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	#msg
 )
 catkin_package(
       CATKIN_DEPENDS 
       message_runtime
)
link_directories(${OpenCV_LIBRARY_DIRS})


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${OpenCV_INCLUDE_DIRS}
)
add_executable(cv_template_video src/cv_template_video.cpp)
target_link_libraries(cv_template_video ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(cv_template_file src/cv_template_file.cpp)
target_link_libraries(cv_template_file ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(cv_RGB_GRAY src/cv_RGB_GRAY.cpp)
target_link_libraries(cv_RGB_GRAY ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(video_slidebar src/video_slidebar.cpp)
target_link_libraries(video_slidebar ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(image_blur src/image_blur.cpp)
target_link_libraries(image_blur ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(cv02_marca_agua src/cv02_marca_agua.cpp)
target_link_libraries(cv02_marca_agua ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(cv03_mix_image src/cv03_mix_image.cpp)
target_link_libraries(cv03_mix_image ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(cv05_filtro src/cv05_filtro.cpp)
target_link_libraries(cv05_filtro ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(cv_operaciones src/cv_operaciones.cpp)
target_link_libraries(cv_operaciones ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(cv04_histograma src/cv04_histograma.cpp)
target_link_libraries(cv04_histograma ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(cv06_brillo_contraste src/cv06_brillo_contraste.cpp)
target_link_libraries(cv06_brillo_contraste ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(cv07_filter src/cv07_filter.cpp)
target_link_libraries(cv07_filter ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(cv08_umbralizacion src/cv08_umbralizacion.cpp)
target_link_libraries(cv08_umbralizacion ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(cv09_k-means.cpp src/cv09_k-means.cpp)
target_link_libraries(cv09_k-means.cpp ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(cv10_region_growing src/cv10_region_growing.cpp)
target_link_libraries(cv10_region_growing ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(cv11_ope_morfologicas src/cv11_ope_morfologicas.cpp)
target_link_libraries(cv11_ope_morfologicas ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(cv12_Sobel_Canny src/cv12_Sobel_Canny.cpp)
target_link_libraries(cv12_Sobel_Canny ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(cv13_hough_transform src/cv13_hough_transform.cpp)
target_link_libraries(cv13_hough_transform ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(cv14_hough_circles src/cv14_hough_transform_circles.cpp)
target_link_libraries(cv14_hough_circles ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(cv14_1_procesamiento_de_video src/cv14_1_procesamiento_de_video.cpp)
target_link_libraries(cv14_1_procesamiento_de_video ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(Algoritmo_Triangulacion_Geometrica src/Algoritmo_Triangulacion_Geometrica.cpp)
target_link_libraries(Algoritmo_Triangulacion_Geometrica ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
add_dependencies(Algoritmo_Triangulacion_Geometrica curso_pdi_generate_messages_cpp)

add_executable(landmarks_publisher src/landmarks_publisher.cpp)
target_link_libraries(landmarks_publisher ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
add_dependencies(landmarks_publisher curso_pdi_generate_messages_cpp)

add_executable(cv15_Reconocimiento_de_frutas src/cv15_Reconocimiento_de_frutas.cpp)
target_link_libraries(cv15_Reconocimiento_de_frutas ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(cv16_detector_esquinas src/cv16_detector_esquinas.cpp)
target_link_libraries(cv16_detector_esquinas ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

