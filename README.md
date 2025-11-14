# Air Hockey Robot

This project implements a robot that plays air hockey using computer vision for image capture and analysis.

## Features
- Captures images from camera in grayscale
- Detects puck using thresholding and Hough circle detection
- Converts image coordinates to physical table coordinates (mm)
- Returns puck coordinates (center x, y and radius)
- Configurable table size and puck parameters

## Configuration
Edit `config/config.hpp` to set:
- `CAMERA_INDEX`: Camera device index (default 0)
- `CHESSBOARD_WIDTH` and `CHESSBOARD_HEIGHT`: Chessboard corners (default 9x6)
- `SQUARE_SIZE`: Chessboard square size in mm (default 25.0)
- `TABLE_WIDTH` and `TABLE_HEIGHT`: Image dimensions in pixels
- `PHYSICAL_TABLE_WIDTH` and `PHYSICAL_TABLE_HEIGHT`: Real table dimensions in mm
- `PUCK_RADIUS_MIN` and `PUCK_RADIUS_MAX`: Minimum and maximum expected puck radius
- `PUCK_THRESHOLD`: Threshold value for detecting black puck on white background

## Setup

### Prerequisites
- OpenCV 4.x
- CMake 3.10 or higher
- C++17 compatible compiler

### Building on Windows (Development)
1. Install OpenCV and set the path in CMakeLists.txt if necessary.
2. Create build directory: `mkdir build && cd build`
3. Configure: `cmake ..`
4. Build: `cmake --build .`

### Building on Raspberry Pi
1. Install OpenCV: `sudo apt update && sudo apt install libopencv-dev`
2. Remove or comment out the `set(OpenCV_DIR ...)` line in CMakeLists.txt
3. Follow the same build steps.

### Running
- **Main application**: `./air_hockey_robot` - Captures image, detects puck, and shows coordinates
- **Camera calibration**: `./calibrate_camera` - Calibrates camera using chessboard pattern

### Camera Calibration
Before running the main application, calibrate the camera for accurate undistortion:
1. Print a chessboard pattern (configured corners and square size)
2. Run `./calibrate_camera`
3. Position the chessboard in different orientations and capture at least 10 images
4. Calibration data is saved to `camera_calibration.yml`

The main application will automatically load calibration data if available.

## Camera Setup on Raspberry Pi
- Enable camera: `sudo raspi-config` -> Interfacing Options -> Camera
- Connect Raspberry Pi Camera Module
- Ensure proper permissions for camera access

## Coordinate System
- **Image Coordinates**: Origin at top-left, X increases right, Y increases down
- **Table Coordinates**: Origin at table center, X from -width/2 to +width/2, Y from -height/2 to +height/2 (in mm)
- Puck positions are automatically converted from image pixels to physical table coordinates
- **Calibration**: If camera calibration data exists, points are undistorted for better accuracy
