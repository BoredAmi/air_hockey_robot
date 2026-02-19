cmake_minimum_required(VERSION 3.10)
project(air_hockey_robot)

set(CMAKE_CXX_STANDARD 17)

set(OpenCV_DIR "C:/msys64/mingw64/share/opencv4")


find_package(OpenCV REQUIRED)

if(NOT WIN32)
    
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBCAMERA REQUIRED libcamera)
else()
    add_definitions(-DUSE_LIBCAMERA=false)
    set(LIBCAMERA_LIBRARIES "")
    set(LIBCAMERA_INCLUDE_DIRS "")
endif()

find_package(Eigen3 REQUIRED)


add_executable(air_hockey_robot apps/main.cpp src/capture.cpp src/calibration.cpp src/kalman.cpp src/trajectory.cpp src/movement.cpp)
if(WIN32)
    target_link_libraries(air_hockey_robot ${OpenCV_LIBS} Eigen3::Eigen ws2_32)
else()
    target_link_libraries(air_hockey_robot ${OpenCV_LIBS} Eigen3::Eigen ${LIBCAMERA_LIBRARIES})
endif()
include_directories(
    include
    config
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${LIBCAMERA_INCLUDE_DIRS}
)

add_executable(calibrate_camera apps/calibrate.cpp src/capture.cpp src/calibration.cpp)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} ${LIBCAMERA_LIBRARIES})
include_directories(
    include
    config
    ${OpenCV_INCLUDE_DIRS}
    ${LIBCAMERA_INCLUDE_DIRS}
)

add_executable(test_puck_detection apps/test_puck_detection.cpp src/capture.cpp src/calibration.cpp)
target_link_libraries(test_puck_detection ${OpenCV_LIBS} ${LIBCAMERA_LIBRARIES})
include_directories(
    include
    config
    ${OpenCV_INCLUDE_DIRS}
    ${LIBCAMERA_INCLUDE_DIRS}
)


add_executable(test_trajectory apps/test_trajectory.cpp src/capture.cpp src/calibration.cpp src/kalman.cpp src/trajectory.cpp)
target_link_libraries(test_trajectory ${OpenCV_LIBS} Eigen3::Eigen ${LIBCAMERA_LIBRARIES})
include_directories(
    include
    config
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${LIBCAMERA_INCLUDE_DIRS}
)

add_executable(test_live_detection apps/test_live_detection.cpp src/capture.cpp src/calibration.cpp src/kalman.cpp src/trajectory.cpp)
target_link_libraries(test_live_detection ${OpenCV_LIBS} Eigen3::Eigen ${LIBCAMERA_LIBRARIES})
include_directories(
    include
    config
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${LIBCAMERA_INCLUDE_DIRS}
)

add_executable(benchmark apps/benchmark.cpp src/capture.cpp src/calibration.cpp src/kalman.cpp src/trajectory.cpp)
target_link_libraries(benchmark ${OpenCV_LIBS} Eigen3::Eigen ${LIBCAMERA_LIBRARIES})
include_directories(
    include
    config
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${LIBCAMERA_INCLUDE_DIRS}
)
add_executable(test_udp apps/test_udp.cpp src/movement.cpp)
if(WIN32)
    target_link_libraries(test_udp ${OpenCV_LIBS} Eigen3::Eigen ws2_32)
else()
    target_link_libraries(test_udp ${OpenCV_LIBS} Eigen3::Eigen)
endif()
include_directories(
    include
    config
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
