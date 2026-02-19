# Air Hockey Robot

An autonomous air hockey robot system that uses computer vision to detect the puck, predict its trajectory, and control a robotic arm to defend the goal. Built with C++ for real-time performance.

## Features

- **Real-time Puck Detection**: Captures images from a camera, applies thresholding and Hough circle detection to locate the puck.
- **Coordinate Conversion**: Transforms image coordinates to physical table coordinates (in mm) for accurate positioning.
- **Trajectory Prediction**: Uses Kalman filtering to predict puck path, including entry points to the defense zone and bounce reflections.
- **Robot Control**: Sends movement commands to an ABB robotic arm via UDP for defensive positioning.
- **Camera Calibration**: Supports chessboard-based calibration for lens distortion correction.
- **Modular Design**: Separate components for capture, detection, prediction, and control, with configurable parameters.

## Requirements

### Software Dependencies
- **OpenCV 4.x**: For image processing and computer vision.
- **Eigen 3.x**: For matrix operations in Kalman filtering.
- **Boost**: For UDP communication (Asio library).
- **CMake 3.10+**: For building the project.
- **C++17 Compatible Compiler**: GCC, Clang, or MSVC.

### Hardware Requirements
- **Camera**: USB camera for puck detection.
- **ABB Robotic Arm**: Compatible with RAPID programming (e.g., IRB 120 series) for movement control.
- **Air Hockey Table**: Physical setup with known dimensions for coordinate mapping.
- **Computer/Raspberry Pi**: For running the vision and control software.

## Installation and Setup

### Building on Windows (Development)
1. Install OpenCV, Eigen, and Boost via vcpkg or manually.
2. Clone the repository and navigate to the project directory.
3. Create a build directory: `mkdir build && cd build`
4. Configure with CMake: `cmake ..`
5. Build: `cmake --build . --config Release`

### Building on Raspberry Pi (Deployment)
1. Update system: `sudo apt update && sudo apt upgrade`
2. Install dependencies: `sudo apt install libopencv-dev libeigen3-dev libboost-all-dev cmake build-essential`
3. Comment out OpenCV_DIR in CMakeLists.txt if using system-installed OpenCV.
4. Follow the same build steps as Windows.

### Camera Calibration
Calibrate the camera for accurate undistortion:
1. Print a chessboard pattern (9x6 corners, 25mm squares by default).
2. Run `./calibrate_camera`.
3. Capture at least 10 images of the chessboard in different positions/orientations.
4. Calibration data saves to `camera_calibration.yml` (loaded automatically by the main app).

## Usage

### Applications
- **`./air_hockey_robot`**: Main application - runs full system with live detection, prediction, and robot control.
- **`./calibrate_camera`**: Camera calibration tool.
- **`./benchmark`**: Performance benchmarking for detection and prediction.
- **`./test_puck_detection`**: Test puck detection on static images.
- **`./test_trajectory`**: Test trajectory prediction.
- **`./test_live_detection`**: Test real-time detection and prediction.
- **`./test_udp`**: Test UDP communication with the robot.

### Running the System
1. Ensure camera and robot are connected and powered.
2. Load the RAPID module on the ABB controller.
3. Run `./air_hockey_robot` to start autonomous play.
4. The robot will detect the puck, predict its path, and move to intercept.

### Configuration
Edit `config/config.hpp` for parameters:
- Camera settings: `CAMERA_INDEX`, resolution.
- Table dimensions: `PHYSICAL_TABLE_WIDTH`, `PHYSICAL_TABLE_HEIGHT`.
- Puck detection: `PUCK_THRESHOLD`, radius ranges.
- Kalman filter: Process/measurement noise, prediction steps.
- Robot control: UDP IP/port, movement speeds.

## Testing

Run individual tests to validate components:
- Puck detection: `./test_puck_detection` (uses images in `img/`).
- Trajectory: `./test_trajectory`.
- Live system: `./test_live_detection`.
- Networking: `./test_udp` (requires robot connection).

Use `./benchmark` to measure frame rates and optimize performance.

## Troubleshooting

- **Camera Issues**: Check device index in config; ensure permissions on Raspberry Pi.
- **Calibration Errors**: Verify chessboard pattern matches config; capture more images.
- **Robot Communication**: Confirm IP/port in config; check firewall/UDP settings.
- **Performance**: Reduce image resolution or increase thresholds for faster processing.
- **Build Errors**: Ensure all dependencies are installed; check CMake output for missing libraries.

## Project Structure

- `src/`: Core source files (capture, kalman, movement, etc.).
- `include/`: Header files.
- `apps/`: Executable applications and tests.
- `config/`: Configuration header.
- `img/`: Sample images for testing.
- `robot/`: ABB RAPID modules.
