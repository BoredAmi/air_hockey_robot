#include "capture.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

int main() {
    std::cout << "Starting puck detection test..." << std::endl;
    // ImageCapture capture(0);  // Not needed for image loading
    // if (!capture.initialize()) {
    //     std::cout << "Failed to initialize capture." << std::endl;
    //     return -1;
    // }

    // Create a dummy capture object or use static method if possible
    // For now, assume detectPuck is static or create instance without init
    ImageCapture capture(0);  // Create without initializing camera

    std::vector<std::string> filenames;
    cv::glob("C:\\Users\\azmoz\\Documents\\inlader\\air_hockey_robot\\img\\*", filenames);  // Load all files from img/ folder

    std::cout << "Found " << filenames.size() << " files" << std::endl;
    for (const auto& f : filenames) {
        std::cout << f << std::endl;
    }

    if (filenames.empty()) {
        std::cout << "No images found in img/ folder." << std::endl;
        return -1;
    }

    for (const auto& filename : filenames) {
        cv::Mat img = cv::imread(filename);
        if (img.empty()) {
            std::cerr << "Failed to load image: " << filename << std::endl;
            continue;
        }

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        cv::Point2f puckCenter = capture.detectPuck(gray);

        if (puckCenter.x >= 0 && puckCenter.y >= 0) {
            // Draw a circle at the detected center (assume radius 20 pixels for visualization)
            cv::circle(img, puckCenter, 20, cv::Scalar(0, 255, 0), 2);  // Green circle
            cv::putText(img, "Puck", puckCenter + cv::Point2f(25, 0), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        } else {
            cv::putText(img, "No Puck Detected", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow("Detected Puck", img);
        std::cout << "Displaying: " << filename << std::endl;
        cv::waitKey(0);  // Wait for key press to show next image
    }

    cv::destroyAllWindows();
    return 0;
}