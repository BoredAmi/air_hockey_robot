#include "capture.hpp"
#include "trajectory.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <vector>

int main() {
    cv::setUseOptimized(true);
    ImageCapture capture(1);  // Use virtual camera index
    if (!capture.initialize()) {
        std::cerr << "Failed to initialize camera." << std::endl;
        return -1;
    }

    cv::namedWindow("Benchmark Puck Detection", cv::WINDOW_NORMAL);
    cv::resizeWindow("Benchmark Puck Detection", 1280, 720);

    TrajectoryPredictor predictor;

    bool running = true;
    bool paused = false;

    std::cout << "Benchmarking puck detection and prediction..." << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  Space: Pause/Resume" << std::endl;
    std::cout << "  Q: Quit" << std::endl;

    // Benchmarking variables
    std::vector<double> detectionTimes;
    std::vector<double> predictionTimes;
    int frameCount = 0;
    const int maxFrames = 100;  // Run for 100 frames

    while (running && frameCount < maxFrames) {
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

            // Benchmark detection
            auto detectStart = std::chrono::high_resolution_clock::now();
            cv::Point2f puckCenter = capture.detectPuck(gray);
            auto detectEnd = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> detectDuration = detectEnd - detectStart;

            bool puckDetected = (puckCenter.x >= 0 && puckCenter.y >= 0);

            // Overlay puck detection
            if (puckDetected) {
                cv::circle(frame, puckCenter, 10, cv::Scalar(0, 255, 0), -1);  // Green circle
                cv::putText(frame, "Puck", puckCenter + cv::Point2f(15, 0), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

                PuckPosition puckPos = {capture.imageToTableCoordinates(puckCenter, capture.getCroppedWidth(), capture.getCroppedHeight()), currentTimeUs};
                
                predictor.addMeasurement(puckPos);

                // Benchmark prediction
                auto predictStart = std::chrono::high_resolution_clock::now();
                uint64_t futureTimeUs = currentTimeUs + 500000;  // 500ms
                cv::Point2f predicted = predictor.predictPosition(futureTimeUs);
                predicted = capture.TableToImageCoordinates(predicted, capture.getCroppedWidth(), capture.getCroppedHeight());
                auto predictEnd = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> predictDuration = predictEnd - predictStart;

                detectionTimes.push_back(detectDuration.count());
                predictionTimes.push_back(predictDuration.count());

                if (predicted.x >= 0 && predicted.y >= 0) {
                    cv::circle(frame, predicted, 10, cv::Scalar(0, 0, 255), -1);  // Red circle
                    cv::putText(frame, "Predicted", predicted + cv::Point2f(15, 0), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
                    
                    // Draw velocity vector as arrow from current to predicted
                    cv::arrowedLine(frame, puckCenter, predicted, cv::Scalar(255, 255, 0), 2, cv::LINE_AA, 0, 0.1);  // Yellow arrow
                }
            }

            cv::imshow("Benchmark Puck Detection", frame);
            frameCount++;
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

    // Print benchmark results
    if (!detectionTimes.empty()) {
        double avgDetect = 0.0;
        for (double t : detectionTimes) avgDetect += t;
        avgDetect /= detectionTimes.size();

        double avgPredict = 0.0;
        for (double t : predictionTimes) avgPredict += t;
        avgPredict /= predictionTimes.size();

        std::cout << "\nBenchmark Results (over " << detectionTimes.size() << " detections):" << std::endl;
        std::cout << "Average Detection Time: " << avgDetect << " ms" << std::endl;
        std::cout << "Average Prediction Time: " << avgPredict << " ms" << std::endl;
        std::cout << "Total Time per Frame (detect + predict): " << (avgDetect + avgPredict) << " ms" << std::endl;
    } else {
        std::cout << "No puck detections during benchmark." << std::endl;
    }

    return 0;
}