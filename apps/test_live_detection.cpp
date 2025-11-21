#include "capture.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    ImageCapture capture(1);  // Use virtual camera index
    if (!capture.initialize()) {
        std::cerr << "Failed to initialize camera." << std::endl;
        return -1;
    }

    cv::namedWindow("Live Puck Detection", cv::WINDOW_NORMAL);
    cv::resizeWindow("Live Puck Detection", 1280, 720);

    bool running = true;
    bool paused = false;

    std::cout << "Controls:" << std::endl;
    std::cout << "  Space: Pause/Resume" << std::endl;
    std::cout << "  Q: Quit" << std::endl;

    while (running) {
        if (!paused) {
            cv::Mat frame = capture.captureImage();
            if (frame.empty()) {
                std::cerr << "Failed to capture frame." << std::endl;
                continue;
            }

            cv::Mat gray = capture.captureGrayscaleImage();
            cv::Point2f puckCenter = capture.detectPuck(gray);

            // Overlay puck detection
            if (puckCenter.x >= 0 && puckCenter.y >= 0) {
                cv::circle(frame, puckCenter, 10, cv::Scalar(0, 255, 0), -1);  // Green circle
                cv::putText(frame, "Puck", puckCenter + cv::Point2f(15, 0), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            }

            cv::imshow("Live Puck Detection", frame);
        }

        int key = cv::waitKey(30);  // 30ms delay
        if (key == ' ') {  // Space to pause/resume
            paused = !paused;
            std::cout << (paused ? "Paused" : "Resumed") << std::endl;
        } else if (key == 'q' || key == 'Q') {  // Q to quit
            running = false;
        }
    }

    cv::destroyAllWindows();
    return 0;
}