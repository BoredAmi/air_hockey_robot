#include "calibration.hpp"
#include <iostream>
#include <string>

int main() {
    CameraCalibration calibrator; // Uses CAMERA_INDEX from config

    if (!calibrator.initialize()) {
        std::cerr << "Failed to initialize camera for calibration." << std::endl;
        return 1;
    }

    std::cout << "Camera calibration started." << std::endl;
    std::cout << "Position a chessboard (9x6) in front of the camera and press Enter to capture images." << std::endl;
    std::cout << "Need at least 10 images from different angles. Press 'q' to finish capturing." << std::endl;

    char key = ' ';
    while (key != 'q') {
        std::cout << "Press Enter to capture calibration image, or 'q' to finish: ";
        std::cin.get(key);
        if (key == '\n') {
            if (calibrator.captureCalibrationImage()) {
                std::cout << "Image captured successfully." << std::endl;
            } else {
                std::cout << "Failed to capture calibration image." << std::endl;
            }
        }
    }

    std::cout << "Performing camera calibration..." << std::endl;
    if (calibrator.calibrateCamera()) {
        std::string filename = "camera_calibration.yml";
        if (calibrator.saveCalibration(filename)) {
            std::cout << "Calibration completed and saved to " << filename << std::endl;
        } else {
            std::cerr << "Failed to save calibration." << std::endl;
            return 1;
        }
    } else {
        std::cerr << "Calibration failed." << std::endl;
        return 1;
    }

    return 0;
}