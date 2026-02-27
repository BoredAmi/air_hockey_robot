#include "capture.hpp"
#include "trajectory.hpp"
#include "movement.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <iostream>
#include <string>

int main() {
    Config config;
    config.loadFromFile();
    bool TableFound = false;
    cv::setUseOptimized(true);
    ImageCapture capture(config);  
    if (!capture.initialize()) {
        std::cerr << "Failed to initialize camera." << std::endl;
        return -1;
    }

    TrajectoryPredictor predictor(config);
    MovementController mover(config);

    cv::namedWindow("Air Hockey Defense", cv::WINDOW_NORMAL);
    cv::resizeWindow("Air Hockey Defense", 1280, 720);

    cv::namedWindow("Parameter Controls", cv::WINDOW_NORMAL);
    cv::resizeWindow("Parameter Controls", 400, 600);

    cv::namedWindow("Threshold Preview", cv::WINDOW_NORMAL);
    cv::resizeWindow("Threshold Preview", 640, 480);

    // Slider variables
    int puck_threshold = config.PUCK_THRESHOLD;
    int puck_min_area = config.PUCK_MIN_AREA;
    int puck_max_area = config.PUCK_MAX_AREA;
    int defense_zone_height = (int)(config.DEFENSE_ZONE_HEIGHT * 10);
    int defense_zone_width = (int)(config.DEFENSE_ZONE_WIDTH * 10);
    int where_defense_zone = config.WHERE_DEFENSE_ZONE;
    int robot_origin_corner = config.robot_origin_corner;

    // Trackbars for parameters
    cv::createTrackbar("PUCK_THRESHOLD", "Parameter Controls", &puck_threshold, 255);
    cv::createTrackbar("PUCK_MIN_AREA", "Parameter Controls", &puck_min_area, 10000);
    cv::createTrackbar("PUCK_MAX_AREA", "Parameter Controls", &puck_max_area, 100000);
    cv::createTrackbar("DEFENSE_ZONE_HEIGHT*10", "Parameter Controls", &defense_zone_height, 5000);  // 0-500 mm
    cv::createTrackbar("DEFENSE_ZONE_WIDTH*10", "Parameter Controls", &defense_zone_width, 5000);
    cv::createTrackbar("WHERE_DEFENSE_ZONE", "Parameter Controls", &where_defense_zone, 3);
    cv::createTrackbar("ROBOT_ORIGIN_CORNER", "Parameter Controls", &robot_origin_corner, 3);

    std::cout << "Starting air hockey robot defense..." << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  Q: Quit" << std::endl;
    std::cout << "  F: Set table found" << std::endl;
    std::cout << "  L: Set table not found" << std::endl;
    std::cout << "  S: Save config" << std::endl;
    std::cout << "  R: Reload config (updates sliders)" << std::endl;
    std::cout << "  D: Reset to default values" << std::endl;
    std::cout << "Use Parameter Controls window to adjust settings." << std::endl;
    std::cout << "Threshold Preview window shows real-time thresholding result." << std::endl;
    std::cout << "Robot origin corner is visualized with a white circle and coordinate axes in the main window." << std::endl;

    bool running = true;

    while (running) {
        // Update config from sliders
        config.PUCK_THRESHOLD = puck_threshold;
        config.PUCK_MIN_AREA = puck_min_area;
        config.PUCK_MAX_AREA = puck_max_area;
        config.DEFENSE_ZONE_HEIGHT = defense_zone_height / 10.0f;
        config.DEFENSE_ZONE_WIDTH = defense_zone_width / 10.0f;
        if (config.WHERE_DEFENSE_ZONE != where_defense_zone) {
            config.WHERE_DEFENSE_ZONE = where_defense_zone;
            predictor.setDefenseZone(where_defense_zone);
        }
        config.robot_origin_corner = robot_origin_corner;
        cv::Mat frame = capture.captureImage();
        if (frame.empty()) {
            std::cerr << "Failed to capture frame." << std::endl;
            continue;
        }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Show threshold preview
        cv::Mat thresh;
        cv::threshold(gray, thresh, config.PUCK_THRESHOLD, 255, cv::THRESH_BINARY_INV);
        cv::imshow("Threshold Preview", thresh);

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
        //Draw every possible defense zone
        cv::Point2f zoneTopLeft, zoneBottomRight;
        TrajectoryPredictor possible_predictor(config);
        for (int i = 0; i < 4; i++) {
            possible_predictor.setDefenseZone(i);
            zoneTopLeft = capture.TableToImageCoordinates(cv::Point2f(possible_predictor.getDefenseZoneXMin(), possible_predictor.getDefenseZoneYMin()), capture.getCroppedWidth(), capture.getCroppedHeight());
            zoneBottomRight = capture.TableToImageCoordinates(cv::Point2f(possible_predictor.getDefenseZoneXMax(), possible_predictor.getDefenseZoneYMax()), capture.getCroppedWidth(), capture.getCroppedHeight());
            cv::rectangle(frame, zoneTopLeft, zoneBottomRight, cv::Scalar(0, 255, 0), 1);  // Green rectangle for each zone
            cv::putText(frame, std::to_string(i), zoneBottomRight + cv::Point2f(-20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }

        // Draw chosen defense zone
        cv::Point2f chosenZoneTopLeft = capture.TableToImageCoordinates(cv::Point2f(predictor.getDefenseZoneXMin(), predictor.getDefenseZoneYMin()), capture.getCroppedWidth(), capture.getCroppedHeight());
        cv::Point2f chosenZoneBottomRight = capture.TableToImageCoordinates(cv::Point2f(predictor.getDefenseZoneXMax(), predictor.getDefenseZoneYMax()), capture.getCroppedWidth(), capture.getCroppedHeight());
        cv::rectangle(frame, chosenZoneTopLeft, chosenZoneBottomRight, cv::Scalar(0, 0, 255), 2);  // Red rectangle for chosen zone

        
        // Draw corners of the table
        cv::Point2f topLeft = capture.TableToImageCoordinates(cv::Point2f(0, 0), capture.getCroppedWidth(), capture.getCroppedHeight());
        cv::Point2f topRight = capture.TableToImageCoordinates(cv::Point2f(config.PHYSICAL_TABLE_WIDTH, 0), capture.getCroppedWidth(), capture.getCroppedHeight());
        cv::Point2f bottomLeft = capture.TableToImageCoordinates(cv::Point2f(0, config.PHYSICAL_TABLE_HEIGHT), capture.getCroppedWidth(), capture.getCroppedHeight());
        cv::Point2f bottomRight = capture.TableToImageCoordinates(cv::Point2f(config.PHYSICAL_TABLE_WIDTH, config.PHYSICAL_TABLE_HEIGHT), capture.getCroppedWidth(), capture.getCroppedHeight());
        
        //add border around picture to make it easier to see the corners and predicted entry point 

        cv::copyMakeBorder(frame, frame, 50, 50, 50, 50, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        
        cv::circle(frame, topLeft+cv::Point2f(50, 50), 5, cv::Scalar(255, 255, 0), -1);  // every corner in different color with label
        cv::circle(frame, topRight+cv::Point2f(50, 50), 5, cv::Scalar(255, 0, 255), -1);
        cv::circle(frame, bottomLeft+cv::Point2f(50, 50), 5, cv::Scalar(0, 255, 255), -1);
        cv::circle(frame, bottomRight+cv::Point2f(50, 50), 5, cv::Scalar(0, 0, 255), -1);
        cv::putText(frame, "TL", topLeft + cv::Point2f(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
        cv::putText(frame, "TR", topRight + cv::Point2f(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
        cv::putText(frame, "BL", bottomLeft + cv::Point2f(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        cv::putText(frame, "BR", bottomRight + cv::Point2f(20, 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);

        // Draw table width and height indicators
        cv::line(frame, topLeft + cv::Point2f(50, 50), topRight + cv::Point2f(50, 50), cv::Scalar(255, 255, 0), 2);
        cv::putText(frame, "Table Width", (topLeft + topRight) * 0.5f + cv::Point2f(50, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
        
        cv::line(frame, topLeft + cv::Point2f(50, 50), bottomLeft + cv::Point2f(50, 50), cv::Scalar(255, 0, 255), 2);
        cv::putText(frame, "Table Height", (topLeft + bottomLeft) * 0.5f + cv::Point2f(-120, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);

        // Visualize robot origin corner
        cv::Point2f robotOriginPoint;
        std::string robotOriginLabel;
        cv::Point2f axisXEnd, axisYEnd;
        switch (config.robot_origin_corner) {
        case 0: // top-left
            robotOriginPoint = topLeft + cv::Point2f(50, 50);
            robotOriginLabel = "Robot Origin (TL)";
            axisXEnd = robotOriginPoint + cv::Point2f(0, 100);  
            axisYEnd = robotOriginPoint + cv::Point2f(100, 0);  
            break;
        case 1: // top-right
            robotOriginPoint = topRight + cv::Point2f(50, 50);
            robotOriginLabel = "Robot Origin (TR)";
            axisXEnd = robotOriginPoint + cv::Point2f(-100, 0); 
            axisYEnd = robotOriginPoint + cv::Point2f(0, 100); 
            break;
        case 2: // bottom-left
            robotOriginPoint = bottomLeft + cv::Point2f(50, 50);
            robotOriginLabel = "Robot Origin (BL)";
            axisXEnd = robotOriginPoint + cv::Point2f(100, 0); 
            axisYEnd = robotOriginPoint + cv::Point2f(0, -100); 
            break;
        case 3: // bottom-right
            robotOriginPoint = bottomRight + cv::Point2f(50, 50);
            robotOriginLabel = "Robot Origin (BR)";
            axisXEnd = robotOriginPoint + cv::Point2f(0, -100) ;  
            axisYEnd = robotOriginPoint + cv::Point2f(-100, 0); 
            break;
        default:
            robotOriginPoint = topLeft + cv::Point2f(50, 50);
            robotOriginLabel = "Robot Origin (TL)";
            axisXEnd = robotOriginPoint + cv::Point2f(0, 100);  
            axisYEnd = robotOriginPoint + cv::Point2f(100, 0);  
            break;
        }
        cv::circle(frame, robotOriginPoint, 15, cv::Scalar(255, 255, 255), 3);  // White circle with thick border
        cv::putText(frame, robotOriginLabel, robotOriginPoint + cv::Point2f(20, -10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        
        // Draw coordinate axes
        cv::arrowedLine(frame, robotOriginPoint, axisXEnd, cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.1); // Red X axis
        cv::arrowedLine(frame, robotOriginPoint, axisYEnd, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.1); // Green Y axis
        cv::putText(frame, "X", axisXEnd + cv::Point2f(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
        cv::putText(frame, "Y", axisYEnd + cv::Point2f(-15, 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        
        if (puckDetected) {
            // Draw puck
            cv::circle(frame, puckCenter+cv::Point2f(50,50), 10, cv::Scalar(0, 255, 0), -1);  // Green circle
            cv::putText(frame, "Puck", puckCenter + cv::Point2f(65, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }

        if (predictedEntryTable.x >= 0 && predictedEntryTable.y >= 0) {
            
            // Draw predicted entry
            cv::Point2f predictedImage = capture.TableToImageCoordinates(predictedEntryTable, capture.getCroppedWidth(), capture.getCroppedHeight());
            cv::circle(frame, predictedImage+cv::Point2f(50,50), 10, cv::Scalar(0, 0, 255), -1);  // Red circle
            cv::putText(frame, "Predicted Entry", predictedImage + cv::Point2f(65, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);

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
        else if (key == 's' || key == 'S') {
            config.saveToFile();
        }
        else if (key == 'd' || key == 'D') {
            config.resetToDefaults();
            // Update slider variables from default config
            puck_threshold = config.PUCK_THRESHOLD;
            puck_min_area = config.PUCK_MIN_AREA;
            puck_max_area = config.PUCK_MAX_AREA;
            defense_zone_height = (int)(config.DEFENSE_ZONE_HEIGHT * 10);
            defense_zone_width = (int)(config.DEFENSE_ZONE_WIDTH * 10);
            where_defense_zone = config.WHERE_DEFENSE_ZONE;
            robot_origin_corner = config.robot_origin_corner;
            // Update trackbar positions
            cv::setTrackbarPos("PUCK_THRESHOLD", "Parameter Controls", puck_threshold);
            cv::setTrackbarPos("PUCK_MIN_AREA", "Parameter Controls", puck_min_area);
            cv::setTrackbarPos("PUCK_MAX_AREA", "Parameter Controls", puck_max_area);
            cv::setTrackbarPos("DEFENSE_ZONE_HEIGHT*10", "Parameter Controls", defense_zone_height);
            cv::setTrackbarPos("DEFENSE_ZONE_WIDTH*10", "Parameter Controls", defense_zone_width);
            cv::setTrackbarPos("WHERE_DEFENSE_ZONE", "Parameter Controls", where_defense_zone);
            cv::setTrackbarPos("ROBOT_ORIGIN_CORNER", "Parameter Controls", robot_origin_corner);
            // Update predictor with default defense zone
            predictor.setDefenseZone(where_defense_zone);
            std::cout << "Config reset to defaults and sliders updated." << std::endl;
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