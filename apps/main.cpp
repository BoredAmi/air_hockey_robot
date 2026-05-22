#include "capture.hpp"
#include "trajectory.hpp"
#include "movement.hpp"
#include "game_controller.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <ctime>

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
    GameController gameController(config);


    bool running = true;

    // Last valid puck measurement (in table coordinates) used to detect jumps
    cv::Point2f lastPuckTablePos(-1, -1);
    uint64_t lastPuckTimeUs = 0;
    bool lastPuckValid = false;
    const double MAX_PUCK_SPEED_MM_S = 3000.0; // threshold to ignore samples (mm/s)
    const double MIN_PUCK_SPEED_MM_S = 30.0; // if slower than this, consider it standing still (mm/s)
    int debugImageIndex = 0; // sequential index for saved debug images

    // FPS counter variables
    int frameCount = 0;
    auto lastTime = std::chrono::high_resolution_clock::now();
    double fps = 0.0;

    while (running) {
        cv::Mat frame = capture.captureImage();
        if (frame.empty()) {
            std::cerr << "Failed to capture frame." << std::endl;
            continue;
        }

        // FPS calculation
        frameCount++;
        auto currentTime_FPS = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = currentTime_FPS - lastTime;
        if (elapsed.count() >= 1.0) {
            fps = frameCount / elapsed.count();
            frameCount = 0;
            lastTime = currentTime_FPS;
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
            cv::Point2f currentTablePos = capture.imageToTableCoordinates(puckCenter, capture.getCroppedWidth(), capture.getCroppedHeight());

            bool acceptSample = true;
            double computedSpeed = 0.0;
            if (lastPuckValid) {
                double dt = (currentTimeUs - lastPuckTimeUs) / 1000000.0; // seconds
                if (dt > 0) {
                    double dx = currentTablePos.x - lastPuckTablePos.x;
                    double dy = currentTablePos.y - lastPuckTablePos.y;
                    computedSpeed = std::hypot(dx, dy) / dt; // mm/s
                    if (computedSpeed > MAX_PUCK_SPEED_MM_S) {
                        acceptSample = false;
                        // std::cout << "Skipping sample: high speed " << computedSpeed << " mm/s" << std::endl;
                    } else if (computedSpeed < MIN_PUCK_SPEED_MM_S) {
                        // Puck nearly still -> reset and reinitialize with zero velocity immediately
                        predictor.reset();
                        PuckPosition puckPos = {currentTablePos, currentTimeUs};
                        predictor.addMeasurement(puckPos);
                        lastPuckTablePos = currentTablePos;
                        lastPuckTimeUs = currentTimeUs;
                        lastPuckValid = true;
                        acceptSample = false;
                    }
                }
            }

            PuckPosition puckPos = {currentTablePos, currentTimeUs};
            if (acceptSample) {
                predictor.addMeasurement(puckPos);
                lastPuckTablePos = currentTablePos;
                lastPuckTimeUs = currentTimeUs;
                lastPuckValid = true;
            }

            predictedEntryTable = predictor.predictEntryToDefenseZone(currentTimeUs);

            cv::Point2f predictedShort = predictor.predictPosition(currentTimeUs + 100000); // +100ms
            double velocityConfidence = predictor.getVelocityConfidence();
            double vx = 0.0, vy = 0.0, speed = 0.0;
            if (predictedShort.x >= 0 && predictedShort.y >= 0) {
                vx = (predictedShort.x - puckPos.position.x) * 10.0;
                vy = (predictedShort.y - puckPos.position.y) * 10.0;
                speed = std::hypot(vx, vy);
            }


            if (!predictor.isInDefenseZone(puckPos.position)) {
                if (velocityConfidence <= 0.3) {
                    
                }
            }
        }


        if (predictedEntryTable.x >= 0 && predictedEntryTable.y >= 0) {
            
            // Check direction: if puck is moving AWAY from defense zone, skip (we already hit it to opponent side)
            bool movingTowardZone = true;
            cv::Point2f predictedShortCheck = predictor.predictPosition(currentTimeUs + 100000);
            if (predictedShortCheck.x >= 0 && predictedShortCheck.y >= 0 && puckDetected) {
                cv::Point2f currentTablePos = capture.imageToTableCoordinates(puckCenter, capture.getCroppedWidth(), capture.getCroppedHeight());
                double vx = (predictedShortCheck.x - currentTablePos.x) * 10.0; // velocity estimate mm/s
                double vy = (predictedShortCheck.y - currentTablePos.y) * 10.0;

                switch (config.WHERE_DEFENSE_ZONE) {
                case 0: // Left zone: check if vx > 0 (moving away from left)
                    if (vx > 0) movingTowardZone = false;
                    break;
                case 1: // Right zone: check if vx < 0 (moving away from right)
                    if (vx < 0) movingTowardZone = false;
                    break;
                case 2: // Bottom zone: check if vy < 0 (moving away from bottom)
                    if (vy < 0) movingTowardZone = false;
                    break;
                case 3: // Top zone: check if vy > 0 (moving away from top)
                    if (vy > 0) movingTowardZone = false;
                    break;
                default:
                    break;
                }
            }

            if (!movingTowardZone) {
                std::cout << "Skipping: puck moving away from defense zone (already hit to opponent side)" << std::endl;
            } else {

            // Move robot
            cv::Point2f robotPosInTable(predictedEntryTable.x, predictedEntryTable.y);
            cv::Point2f robotPos = mover.TableToRobotCoordinates(robotPosInTable);

            if(mover.moveTo(robotPos)) {
                std::cout << "Camera coordinates entry: X: " << predictedEntryTable.x << " mm , Y: " << predictedEntryTable.y << " mm" << std::endl;
                std::cout << "Moving robot to: X: " << robotPos.x << " mm, Y: " << robotPos.y << " mm" << std::endl;

                // Estimate predicted entry time by probing predictor over a short horizon
                uint64_t predictedEntryTimeUs = 0;
                const uint64_t maxLookaheadUs = 2000000; // 2 seconds
                const uint64_t stepUs = 10000; // 10 ms
                for (uint64_t t = currentTimeUs; t <= currentTimeUs + maxLookaheadUs; t += stepUs) {
                    cv::Point2f p = predictor.predictPosition(t);
                    if (p.x < 0 || p.y < 0) continue;
                    if (predictor.isInDefenseZone(p)) {
                        predictedEntryTimeUs = t;
                        break;
                    }
                }

                double timeUntilMs = -1.0;
                if (predictedEntryTimeUs > 0) timeUntilMs = (predictedEntryTimeUs - currentTimeUs) / 1000.0;

                // Render debug image with predictions
                cv::Point2f predictedShort = predictor.predictPosition(currentTimeUs + 100000); // +100ms
                double velocityConfidence = predictor.getVelocityConfidence();
                
                DebugRenderParams debugParams{
                    frame,
                    puckCenter,
                    predictedShort,
                    predictedEntryTable,
                    robotPos,
                    velocityConfidence,
                    fps,
                    currentTimeUs,
                    debugImageIndex,
                    capture,
                    predictor
                };
                gameController.renderDebugImage(debugParams);
            } else {
                std::cout << "Point too close to last position" << std::endl;
            }
            }
        }
    }

    mover.stop();
    std::cout << "Stopped." << std::endl;
    return 0;
}