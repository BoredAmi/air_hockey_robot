cmake_minimum_required(VERSION 3.10)
project(air_hockey_robot)

set(CMAKE_CXX_STANDARD 17)


if(WIN32)
    set(OpenCV_DIR "C:/msys64/mingw64/share/opencv4")
else()
    set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")
endif()

find_package(OpenCV REQUIRED)


find_package(nlohmann_json REQUIRED)

if(NOT WIN32)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBCAMERA REQUIRED libcamera)
else()
    add_definitions(-DUSE_LIBCAMERA=false)
    set(LIBCAMERA_LIBRARIES "")
    set(LIBCAMERA_INCLUDE_DIRS "")
endif()

find_package(Eigen3 REQUIRED)

include_directories(
    include
    config
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${LIBCAMERA_INCLUDE_DIRS}
)


add_executable(air_hockey_robot apps/main.cpp src/capture.cpp src/kalman.cpp src/trajectory.cpp src/movement.cpp)
if(WIN32)
    target_link_libraries(air_hockey_robot ${OpenCV_LIBS} Eigen3::Eigen ws2_32)
else()
    target_link_libraries(air_hockey_robot ${OpenCV_LIBS} Eigen3::Eigen ${LIBCAMERA_LIBRARIES})
endif()
add_executable(preview_app apps/app_with_preview.cpp src/capture.cpp src/kalman.cpp src/trajectory.cpp src/movement.cpp)
if(WIN32)
    target_link_libraries(preview_app ${OpenCV_LIBS} Eigen3::Eigen ws2_32)
else()
    target_link_libraries(preview_app ${OpenCV_LIBS} Eigen3::Eigen ${LIBCAMERA_LIBRARIES})
endif()


add_executable(test_trajectory apps/test_trajectory.cpp src/capture.cpp src/kalman.cpp src/trajectory.cpp)
target_link_libraries(test_trajectory ${OpenCV_LIBS} Eigen3::Eigen ${LIBCAMERA_LIBRARIES})

add_executable(test_live_detection apps/test_live_detection.cpp src/capture.cpp src/kalman.cpp src/trajectory.cpp)
target_link_libraries(test_live_detection ${OpenCV_LIBS} Eigen3::Eigen ${LIBCAMERA_LIBRARIES})

add_executable(benchmark apps/benchmark.cpp src/capture.cpp src/kalman.cpp src/trajectory.cpp src/movement.cpp)
target_link_libraries(benchmark ${OpenCV_LIBS} Eigen3::Eigen ${LIBCAMERA_LIBRARIES})
if(WIN32)
    target_link_libraries(benchmark ws2_32)
endif()

add_executable(camera_preview apps/camera_preview.cpp src/capture.cpp)
target_link_libraries(camera_preview ${OpenCV_LIBS} Eigen3::Eigen ${LIBCAMERA_LIBRARIES})
if(WIN32)
    target_link_libraries(camera_preview ws2_32)
endif()

add_executable(test_udp apps/test_udp.cpp src/movement.cpp)
target_link_libraries(test_udp ${OpenCV_LIBS} Eigen3::Eigen)
if(WIN32)
    target_link_libraries(test_udp ws2_32)
endif()

add_executable(config_tuner apps/config_tuner.cpp src/capture.cpp src/kalman.cpp src/trajectory.cpp src/movement.cpp)
target_link_libraries(config_tuner ${OpenCV_LIBS} Eigen3::Eigen ${LIBCAMERA_LIBRARIES})
if(WIN32)
    target_link_libraries(config_tuner ws2_32)
endif()

add_executable(test_opencv apps/test_opencv.cpp)
target_link_libraries(test_opencv ${OpenCV_LIBS})