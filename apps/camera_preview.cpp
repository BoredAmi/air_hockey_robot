#include "capture.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    Config config;
    config.loadFromFile();

    ImageCapture capture(config);
    if (!capture.initialize()) {
        std::cerr << "Failed to initialize camera." << std::endl;
        return -1;
    }

    cv::namedWindow("Camera Preview (raw, before processing)", cv::WINDOW_NORMAL);
    cv::resizeWindow("Camera Preview (raw, before processing)", 800, 600);

    std::cout << "Camera preview showing RAW image (before any processing)." << std::endl;
    std::cout << "This is exactly what the camera captures, using the same capture methods." << std::endl;
    std::cout << "Press 'q' to quit, 's' to save frame." << std::endl;

    while (true) {
        cv::Mat frame = capture.captureRawImage();
        if (frame.empty()) {
            std::cerr << "Failed to capture frame." << std::endl;
            continue;
        }

        cv::imshow("Camera Preview (raw, before processing)", frame);

        int key = cv::waitKey(1); 
        if (key == 'q' || key == 'Q') {
            break;
        } else if (key == 's' || key == 'S') {
            std::string filename = "raw_preview_" + std::to_string(cv::getTickCount()) + ".png";
            cv::imwrite(filename, frame);
            std::cout << "Frame saved as: " << filename << std::endl;
        }
    }

    cv::destroyAllWindows();
    std::cout << "Preview closed." << std::endl;
    return 0;
}