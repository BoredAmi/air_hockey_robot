#include "capture.hpp"
#include "trajectory.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <limits>

int main() {
    std::cout << "Starting trajectory prediction test..." << std::endl;

    ImageCapture capture(0); 
    TrajectoryPredictor predictor;

    std::vector<std::string> filenames = {"../img/1.png", "../img/2.png", "../img/3.png", "../img/4.png"};
    uint64_t baseTimestamp = 1000000;  // Start at 1 second (microseconds)
    uint64_t timeStep = 100000;  // 100ms between images

    std::vector<cv::Point2f> positions;  // Store positions for plotting

    for (size_t i = 0; i < filenames.size(); ++i) {
        cv::Mat img = cv::imread(filenames[i]);
        if (img.empty()) {
            std::cerr << "Failed to load image: " << filenames[i] << std::endl;
            continue;
        }

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        cv::Point2f puckCenter = capture.detectPuck(gray);
        if (puckCenter.x >= 0 && puckCenter.y >= 0) {
            uint64_t timestamp = baseTimestamp + i * timeStep;

            PuckPosition puckPos = {puckCenter, timestamp};
            predictor.addMeasurement(puckPos);
            positions.push_back(puckCenter);

            std::cout << "Added measurement from " << filenames[i] << ": (" << puckCenter.x << ", " << puckCenter.y << ") at t=" << timestamp << " us" << std::endl;
        } else {
            std::cout << "No puck detected in " << filenames[i] << std::endl;
        }
    }

    // Predict future position, e.g., 200ms ahead
    uint64_t futureTimestamp = baseTimestamp + 4 * timeStep + 100000;  // After last image + 200ms
    cv::Point2f predictedPos = predictor.predictPosition(futureTimestamp);

    if (predictedPos.x != -1) {
        std::cout << "Predicted position at t=" << futureTimestamp << " us: (" << predictedPos.x << ", " << predictedPos.y << ")" << std::endl;
    } else {
        std::cout << "Prediction failed (not initialized or invalid timestamp)." << std::endl;
    }

    // Plot the trajectory
    if (!positions.empty()) {
        // Collect all points for scaling
        std::vector<cv::Point2f> allPoints = positions;
        if (predictedPos.x != -1) {
            allPoints.push_back(predictedPos);
        }

        // Find min and max
        float minX = std::numeric_limits<float>::max();
        float minY = std::numeric_limits<float>::max();
        float maxX = std::numeric_limits<float>::min();
        float maxY = std::numeric_limits<float>::min();
        for (const auto& p : allPoints) {
            minX = std::min(minX, p.x);
            minY = std::min(minY, p.y);
            maxX = std::max(maxX, p.x);
            maxY = std::max(maxY, p.y);
        }

        // Translate so min is at (50, 50)
        float offsetX = 50 - minX;
        float offsetY = 50 - minY;

        // Image size based on range + margins
        int imgWidth = std::max(1280, (int)(maxX - minX + 200));
        int imgHeight = std::max(720, (int)(maxY - minY + 200));
        cv::Mat plotImg(imgHeight, imgWidth, CV_8UC3, cv::Scalar(255, 255, 255));  // White background

        // Draw detected positions
        for (size_t i = 0; i < positions.size(); ++i) {
            cv::Point2f pos = positions[i] + cv::Point2f(offsetX, offsetY);
            cv::circle(plotImg, pos, 10, cv::Scalar(0, 255, 0), -1);  // Green circles
            cv::putText(plotImg, std::to_string(i + 1), pos + cv::Point2f(15, 0), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }

        // Draw trajectory line
        if (positions.size() > 1) {
            for (size_t i = 0; i < positions.size() - 1; ++i) {
                cv::Point2f p1 = positions[i] + cv::Point2f(offsetX, offsetY);
                cv::Point2f p2 = positions[i + 1] + cv::Point2f(offsetX, offsetY);
                cv::line(plotImg, p1, p2, cv::Scalar(255, 0, 0), 2);  // Blue line
            }
        }

        // Draw predicted position
        if (predictedPos.x != -1) {
            cv::Point2f predPos = predictedPos + cv::Point2f(offsetX, offsetY);
            cv::circle(plotImg, predPos, 15, cv::Scalar(0, 0, 255), -1);  // Red circle
            cv::putText(plotImg, "Predicted", predPos + cv::Point2f(20, 0), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        }

        cv::imshow("Trajectory Plot", plotImg);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    return 0;
}