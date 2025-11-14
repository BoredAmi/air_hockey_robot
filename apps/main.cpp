#include "capture.hpp"
#include <iostream>
#include <string>

int main() {
    ImageCapture capturer; // Uses CAMERA_INDEX from config

    if (!capturer.initialize()) {
        std::cerr << "Failed to initialize camera." << std::endl;
        return 1;
    }

    std::cout << "Camera initialized. Capturing grayscale image..." << std::endl;

    cv::Mat grayImage = capturer.captureGrayscaleImage();
    if (grayImage.empty()) {
        std::cerr << "Failed to capture image." << std::endl;
        return 1;
    }

    std::cout << "Detecting puck..." << std::endl;
    std::vector<cv::Vec3f> circles = capturer.detectPuck(grayImage);

    if (circles.empty()) {
        std::cout << "No puck detected." << std::endl;
    } else {
        std::cout << "Detected " << circles.size() << " potential pucks:" << std::endl;
        for (size_t i = 0; i < circles.size(); i++) {
            cv::Vec3f c = circles[i];
            cv::Point2f imageCenter(c[0], c[1]);
            cv::Point2f tablePos = capturer.imageToTableCoordinates(imageCenter, grayImage.cols, grayImage.rows);
            std::cout << "Puck " << i+1 << ": Image Center=(" << c[0] << ", " << c[1] << "), Radius=" << c[2] 
                      << " -> Table Position=(" << tablePos.x << " mm, " << tablePos.y << " mm)" << std::endl;
        }
    }

    // Optionally save the grayscale image
    std::string filename = "captured_gray_image.jpg";
    if (capturer.saveImage(grayImage, filename)) {
        std::cout << "Grayscale image saved as " << filename << std::endl;
    } else {
        std::cerr << "Failed to save grayscale image." << std::endl;
    }

    return 0;
}