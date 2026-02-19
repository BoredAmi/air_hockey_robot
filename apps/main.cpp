#include "capture.hpp"
#include "trajectory.hpp"
#include "movement.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <iostream>
#include <string>

int main() {
    bool TableFound = false;
    cv::setUseOptimized(true);
    ImageCapture capture(CAMERA_INDEX);  
    if (!capture.initialize()) {
        std::cerr << "Failed to initialize camera." << std::endl;
        return -1;
    }

    TrajectoryPredictor predictor;
    MovementController mover;

    cv::namedWindow("Air Hockey Defense", cv::WINDOW_NORMAL);
    cv::resizeWindow("Air Hockey Defense", 1280, 720);

    std::cout << "Starting air hockey robot defense..." << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  Q: Quit" << std::endl;

    bool running = true;

    while (running) {
        cv::Mat frame = capture.captureImage();
        if (frame.empty()) {
            std::cerr << "Failed to capture frame." << std::endl;
            continue;
        }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        cv::Point2f puckCenter = capture.detectPuck(gray);
        bool puckDetected = (puckCenter.x >= 0 && puckCenter.y >= 0);

        double currentTime = cv::getTickCount() / cv::getTickFrequency();
        uint64_t currentTimeUs = (uint64_t)(currentTime * 1000000.0);

        cv::Point2f predictedEntryTable;
        predictedEntryTable.x = -1.0f;  // Initialize to invalid position
        predictedEntryTable.y = -1.0f;
        if (puckDetected) {
            PuckPosition puckPos = {capture.imageToTableCoordinates(puckCenter, capture.getCroppedWidth(), capture.getCroppedHeight()), currentTimeUs};
            predictor.addMeasurement(puckPos);

            // Only predict if puck is outside defense zone 
            if (!predictor.isInDefenseZone(puckPos.position)) {
                // Predict entry to defense zone
                predictedEntryTable = predictor.predictEntryToDefenseZone(currentTimeUs);
            }
        }

        // Draw defense zone

        
        cv::Point2f zoneTopLeft = capture.TableToImageCoordinates(cv::Point2f(predictor.zoneXMin, predictor.zoneYMin), capture.getCroppedWidth(), capture.getCroppedHeight());
        cv::Point2f zoneBottomRight = capture.TableToImageCoordinates(cv::Point2f(predictor.zoneXMax, predictor.zoneYMax), capture.getCroppedWidth(), capture.getCroppedHeight());
        cv::rectangle(frame, zoneTopLeft, zoneBottomRight, cv::Scalar(255, 0, 0), 2);  // Blue rectangle for zone

        // Draw corners of the table
        cv::Point2f topLeft = capture.TableToImageCoordinates(cv::Point2f(0, 0), capture.getCroppedWidth(), capture.getCroppedHeight());
        cv::Point2f topRight = capture.TableToImageCoordinates(cv::Point2f(PHYSICAL_TABLE_WIDTH, 0), capture.getCroppedWidth(), capture.getCroppedHeight());
        cv::Point2f bottomLeft = capture.TableToImageCoordinates(cv::Point2f(0, PHYSICAL_TABLE_HEIGHT), capture.getCroppedWidth(), capture.getCroppedHeight());
        cv::Point2f bottomRight = capture.TableToImageCoordinates(cv::Point2f(PHYSICAL_TABLE_WIDTH, PHYSICAL_TABLE_HEIGHT), capture.getCroppedWidth(), capture.getCroppedHeight());
        cv::circle(frame, topLeft, 5, cv::Scalar(255, 255, 0), -1);  // every corner in different color with label
        cv::circle(frame, topRight, 5, cv::Scalar(255, 0, 255), -1);
        cv::circle(frame, bottomLeft, 5, cv::Scalar(0, 255, 255), -1);
        cv::circle(frame, bottomRight, 5, cv::Scalar(0, 0, 255), -1);
        cv::putText(frame, "Top Left", topLeft + cv::Point2f(15, 0), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
        cv::putText(frame, "Top Right", topRight + cv::Point2f(15, 0), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
        cv::putText(frame, "Bottom Left", bottomLeft + cv::Point2f(15, 0), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        cv::putText(frame, "Bottom Right", bottomRight + cv::Point2f(15, 0), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);

        if (puckDetected) {
            // Draw puck
            cv::circle(frame, puckCenter, 10, cv::Scalar(0, 255, 0), -1);  // Green circle
            cv::putText(frame, "Puck", puckCenter + cv::Point2f(15, 0), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }

        if (predictedEntryTable.x >= 0 && predictedEntryTable.y >= 0) {
            
            // Draw predicted entry
            cv::Point2f predictedImage = capture.TableToImageCoordinates(predictedEntryTable, capture.getCroppedWidth(), capture.getCroppedHeight());
            cv::circle(frame, predictedImage, 10, cv::Scalar(0, 0, 255), -1);  // Red circle
            cv::putText(frame, "Predicted Entry", predictedImage + cv::Point2f(15, 0), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);

            // Move robot
            cv::Point2f robotPosInTable(predictedEntryTable.x, predictedEntryTable.y);
            cv::Point2f robotPos = mover.TableToRobotCoordinates(robotPosInTable);

            mover.moveTo(robotPos);
            
            std::cout << "Moving robot to X: " << predictedEntryTable.x << " mm and Y: " << predictedEntryTable.y << " mm" << std::endl;
            
        }

        cv::imshow("Air Hockey Defense", frame);

        int key = cv::waitKey(30);  // 30ms delay
        if (key == 'f'){
            TableFound = true;
            capture.tableFound(true);
        }
         else if (key == 'l'){
            TableFound = false;
            capture.tableFound(false);
        }
        if (key == 'q' || key == 'Q') {
            running = false;
        }
    }

    cv::destroyAllWindows();
    mover.stop();
    std::cout << "Stopped." << std::endl;
    return 0;
}