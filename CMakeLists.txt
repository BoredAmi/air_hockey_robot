cmake_minimum_required(VERSION 3.10)
project(air_hockey_robot)

set(CMAKE_CXX_STANDARD 17)

# Wskaż katalog z OpenCVConfig.cmake
set(OpenCV_DIR "C:/msys64/mingw64/share/opencv4")

# Znajdź OpenCV
find_package(OpenCV REQUIRED)

# Znajdź Eigen
find_package(Eigen3 REQUIRED)

# Dodaj pliki źródłowe 
add_executable(air_hockey_robot apps/main.cpp src/capture.cpp src/calibration.cpp src/kalman.cpp src/trajectory.cpp)

# Podłącz biblioteki i ścieżki include
target_link_libraries(air_hockey_robot ${OpenCV_LIBS} Eigen3::Eigen)
include_directories(
    include
    config
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(calibrate_camera apps/calibrate.cpp src/capture.cpp src/calibration.cpp)
target_link_libraries(calibrate_camera ${OpenCV_LIBS})
include_directories(
    include
    config
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(test_puck_detection apps/test_puck_detection.cpp src/capture.cpp src/calibration.cpp)
target_link_libraries(test_puck_detection ${OpenCV_LIBS})
include_directories(
    include
    config
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(test_trajectory apps/test_trajectory.cpp src/capture.cpp src/calibration.cpp src/kalman.cpp src/trajectory.cpp)
target_link_libraries(test_trajectory ${OpenCV_LIBS} Eigen3::Eigen)
include_directories(
    include
    config
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(test_live_detection apps/test_live_detection.cpp src/capture.cpp src/calibration.cpp)
target_link_libraries(test_live_detection ${OpenCV_LIBS})
include_directories(
    include
    config
    ${OpenCV_INCLUDE_DIRS}
)
