#include "capture.hpp"
#include "trajectory.hpp"
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

    TrajectoryPredictor predictor;

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

            double currentTime = cv::getTickCount() / cv::getTickFrequency();
            uint64_t currentTimeUs = (uint64_t)(currentTime * 1000000.0);

            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            cv::Point2f puckCenter = capture.detectPuck(gray);

            // Overlay puck detection
            if (puckCenter.x >= 0 && puckCenter.y >= 0) {
                cv::circle(frame, puckCenter, 10, cv::Scalar(0, 255, 0), -1);  // Green circle
                cv::putText(frame, "Puck", puckCenter + cv::Point2f(15, 0), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

                PuckPosition puckPos = {capture.imageToTableCoordinates(puckCenter, capture.getCroppedWidth(), capture.getCroppedHeight()), currentTimeUs};
                
                predictor.addMeasurement(puckPos);
                std::cout << "Detected Puck Position (Table Coords): " << puckPos.position << " at " << puckPos.timestamp << " us" << std::endl;
            }

            // Predict position in 200ms
            uint64_t futureTimeUs = currentTimeUs + 5000000;  // 500ms = 500000 us
            cv::Point2f predicted = predictor.predictPosition(futureTimeUs);
            predicted = capture.TableToImageCoordinates(predicted, capture.getCroppedWidth(), capture.getCroppedHeight());
            std::cout << "Predicted Position (Image Coords): " << predicted << std::endl;
            if (predicted.x >= 0 && predicted.y >= 0) {
                cv::circle(frame, predicted, 10, cv::Scalar(0, 0, 255), -1);  // Red circle
                cv::putText(frame, "Predicted", predicted + cv::Point2f(15, 0), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
                
                // Draw velocity vector as arrow from current to predicted
                if (puckCenter.x >= 0 && puckCenter.y >= 0) {
                    cv::arrowedLine(frame, puckCenter, predicted, cv::Scalar(255, 255, 0), 2, cv::LINE_AA, 0, 0.1);  // Yellow arrow
                }
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