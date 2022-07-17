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

# # Keyframe Testing Module
# add_executable(
#   TestKeyframe
#   CVColor.cpp
#   ImageProcessing.cpp
#   PointPolar.cpp 
#   RadarImageHandler.cpp
#   Keyframe.cpp
#   RadarImage.cpp
#   RegistrationCostFunctor.cpp
#   OptimisationHandler.cpp
#   OrientedSurfacePointsHandler.cpp
#   RadarFeedHandler.cpp
#   RadarFeed.cpp
#   TestKeyframe.cpp
# )
# set_property(TARGET TestKeyframe PROPERTY C_STANDARD 11)
# set_property(TARGET TestKeyframe PROPERTY CXX_STANDARD 17)
# target_link_libraries(TestKeyframe ${OpenCV_LIBS} ${CERES_LIBRARIES})

# # Debugging of Cost Function
# add_executable(
#   TestCostFunction
#   CVColor.cpp
#   ImageProcessing.cpp
#   PointPolar.cpp 
#   RadarImageHandler.cpp
#   Keyframe.cpp
#   RadarImage.cpp
#   RegistrationCostFunctor.cpp
#   OptimisationHandler.cpp
#   OrientedSurfacePointsHandler.cpp
#   RadarFeedHandler.cpp
#   RadarFeed.cpp
#   TestCostFunction.cpp
# )
# set_property(TARGET TestCostFunction PROPERTY C_STANDARD 11)
# set_property(TARGET TestCostFunction PROPERTY CXX_STANDARD 17)
# target_link_libraries(TestCostFunction ${OpenCV_LIBS} ${CERES_LIBRARIES})

# Ground Truth Processing Module
add_executable(
  ProcessGroundTruth
  RadarFeedHandler.cpp
  ProcessGroundTruth.cpp
)
set_property(TARGET ProcessGroundTruth PROPERTY C_STANDARD 11)
set_property(TARGET ProcessGroundTruth PROPERTY CXX_STANDARD 17)
target_link_libraries(ProcessGroundTruth ${OpenCV_LIBS})


# Ground Truth Processing Module
add_executable(
  PosesToKITTI
  BenchmarkHandler.cpp
  PosesToKITTI.cpp
)
set_property(TARGET PosesToKITTI PROPERTY C_STANDARD 11)
set_property(TARGET PosesToKITTI PROPERTY CXX_STANDARD 17)
target_link_libraries(PosesToKITTI ${OpenCV_LIBS})


# Main CFEAR Running Module
add_executable(
  RunCFEAR
  CVColor.cpp
  ImageProcessing.cpp
  PointPolar.cpp 
  RadarImageHandler.cpp
  Keyframe.cpp
  RadarImage.cpp
  RegistrationCostFunctor.cpp
  OptimisationHandler.cpp
  OrientedSurfacePointsHandler.cpp
  RadarFeedHandler.cpp
  RadarFeed.cpp
  RunCFEAR.cpp
)
set_property(TARGET RunCFEAR PROPERTY C_STANDARD 11)
set_property(TARGET RunCFEAR PROPERTY CXX_STANDARD 17)
target_link_libraries(RunCFEAR ${OpenCV_LIBS} ${CERES_LIBRARIES})
