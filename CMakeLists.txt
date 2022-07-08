cmake_minimum_required(VERSION 3.2)

project(TestRadar)

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SparseLinearAlgebraLibrary)
find_package(OpenMP)

include_directories(${Boost_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS)

# Need to disable multithreading
add_compile_definitions(EIGEN_DONT_PARALLELIZE)

# Radar Testing Module
add_executable(
  TestRadar
  CVColor.cpp
  ImageProcessing.cpp
  PointPolar.cpp 
  RadarImageHandler.cpp
  RadarImage.cpp
  Keyframe.cpp
  OrientedSurfacePointsHandler.cpp
  TestRadar.cpp
)
set_property(TARGET TestRadar PROPERTY C_STANDARD 11)
set_property(TARGET TestRadar PROPERTY CXX_STANDARD 17)
target_link_libraries(TestRadar ${OpenCV_LIBS})

# Keyframe Testing Module
add_executable(
  TestKeyframe
  CVColor.cpp
  ImageProcessing.cpp
  PointPolar.cpp 
  RadarImageHandler.cpp
  Keyframe.cpp
  RadarImage.cpp
  RegistrationCostFunctor.cpp
  OptimisationHandler.cpp
  OrientedSurfacePointsHandler.cpp
  OdometryVisualiser.cpp
  RadarFeedHandler.cpp
  RadarFeed.cpp
  TestKeyframe.cpp
)
set_property(TARGET TestKeyframe PROPERTY C_STANDARD 11)
set_property(TARGET TestKeyframe PROPERTY CXX_STANDARD 17)
target_link_libraries(TestKeyframe ${OpenCV_LIBS} ${CERES_LIBRARIES})

# Main Visualiser Testing module
# add_executable(
#   TestVisualiser
#   CVColor.cpp
#   OdometryVisualiser.cpp
#   ImageProcessing.cpp
#   PointPolar.cpp 
#   RadarImageHandler.cpp
#   RadarImage.cpp
#   PoseGraph.cpp
#   RadarFeedHandler.cpp
#   RadarFeed.cpp
#   TestVisualiser.cpp)
# set_property(TARGET TestVisualiser PROPERTY CXX_STANDARD 17)

# OpenMP Linking
# if(OpenMP_CXX_FOUND)
#   target_link_libraries(
#     TestVisualiser PUBLIC OpenMP::OpenMP_C OpenMP::OpenMP_CXX ${OpenCV_LIBS}
#                           ${CERES_LIBRARIES})
# else()
#   target_link_libraries(TestVisualiser ${OpenCV_LIBS} ${CERES_LIBRARIES})
# endif()
