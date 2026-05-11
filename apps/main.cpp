#include "capture.hpp"
#include "trajectory.hpp"
#include "movement.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <filesystem>

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
            if (lastPuckValid) {
                double dt = (currentTimeUs - lastPuckTimeUs) / 1000000.0; // seconds
                if (dt > 0) {
                    double dx = currentTablePos.x - lastPuckTablePos.x;
                    double dy = currentTablePos.y - lastPuckTablePos.y;
                    double speed = std::hypot(dx, dy) / dt; // mm/s
                    if (speed > MAX_PUCK_SPEED_MM_S) {
                        acceptSample = false;
                        std::cout << "Skipping sample: high speed " << speed << " mm/s" << std::endl;
                    } else if (speed < MIN_PUCK_SPEED_MM_S) {
                        // Puck nearly still -> clear predictor and ignore this sample
                        predictor.reset();
                        lastPuckValid = false;
                        acceptSample = false;
                        std::cout << "Resetting predictor: puck nearly still (" << speed << " mm/s)" << std::endl;
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

                // Draw debug overlay and save image so you can inspect remaining time after sending move
                try {
                    cv::Mat debugImg = frame.clone();

                    // Draw puck center
                    cv::circle(debugImg, puckCenter, 6, cv::Scalar(0, 0, 255), -1);

                    // Short-horizon predicted position and velocity
                    cv::Point2f predictedShort = predictor.predictPosition(currentTimeUs + 100000); // +100ms
                    double velocityConfidence = predictor.getVelocityConfidence();
                    double vx = 0.0, vy = 0.0, speed = 0.0;
                    if (predictedShort.x >= 0 && predictedShort.y >= 0) {
                        cv::Point2f predictedShortImg = capture.TableToImageCoordinates(predictedShort, capture.getCroppedWidth(), capture.getCroppedHeight());
                        cv::arrowedLine(debugImg, puckCenter, predictedShortImg, cv::Scalar(0, 255, 255), 2, cv::LINE_AA, 0, 0.2);
                        vx = (predictedShort.x - capture.imageToTableCoordinates(puckCenter, capture.getCroppedWidth(), capture.getCroppedHeight()).x) * 10.0;
                        vy = (predictedShort.y - capture.imageToTableCoordinates(puckCenter, capture.getCroppedWidth(), capture.getCroppedHeight()).y) * 10.0;
                        speed = std::hypot(vx, vy);
                    }

                    // Draw predicted path 
                    uint64_t predictedEntryTimeUs = 0;
                    const uint64_t maxLookaheadUs = 2000000; // 2 seconds
                    const uint64_t stepUs = 50000; // 50 ms
                    std::vector<cv::Point> pathPoints;
                    for (uint64_t t = currentTimeUs; t <= currentTimeUs + maxLookaheadUs; t += stepUs) {
                        cv::Point2f p = predictor.predictPosition(t);
                        if (p.x < 0 || p.y < 0) continue;
                        cv::Point imgPt = capture.TableToImageCoordinates(p, capture.getCroppedWidth(), capture.getCroppedHeight());
                        pathPoints.push_back(imgPt);
                        if (predictor.isInDefenseZone(p) && predictedEntryTimeUs == 0) {
                            predictedEntryTimeUs = t;
                        }
                    }
                    for (size_t i = 1; i < pathPoints.size(); ++i) {
                        cv::line(debugImg, pathPoints[i-1], pathPoints[i], cv::Scalar(0, 255, 0), 1);
                    }

                    // Draw predicted entry point
                    cv::Point2f predictedImage = capture.TableToImageCoordinates(predictedEntryTable, capture.getCroppedWidth(), capture.getCroppedHeight());
                    if (predictedImage.x >= 0 && predictedImage.y >= 0) {
                        cv::circle(debugImg, predictedImage, 8, cv::Scalar(255, 0, 0), 2);
                    }

                    // Put multiple text lines: index, fps, confidence, speed, time-to-entry, predicted/robot coords
                    std::ostringstream line1, line2, line3, line4;
                    line1 << "Idx:" << std::setw(4) << std::setfill('0') << debugImageIndex << "  FPS:" << std::fixed << std::setprecision(1) << fps;
                    line2 << "Conf:" << std::fixed << std::setprecision(2) << velocityConfidence << "  V(mm/s):" << std::fixed << std::setprecision(1) << speed;
                    if (predictedEntryTimeUs > 0) {
                        double timeUntilMs = (predictedEntryTimeUs - currentTimeUs) / 1000.0;
                        line2 << "  Tentry(ms):" << std::fixed << std::setprecision(0) << timeUntilMs;
                    } else {
                        line2 << "  Tentry(ms):unknown";
                    }

                    line3 << "Pred(mm):" << std::fixed << std::setprecision(0) << predictedEntryTable.x << "," << predictedEntryTable.y;
                    line3 << "  Robot(mm):" << std::fixed << std::setprecision(0) << robotPos.x << "," << robotPos.y;

                    line4 << "ShortPred(mm):" << std::fixed << std::setprecision(0) << predictedShort.x << "," << predictedShort.y;

                    int imgX = 10;
                    int y = 30;
                    int lineH = 28;
                    cv::putText(debugImg, line1.str(), cv::Point(imgX, y), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                    cv::putText(debugImg, line2.str(), cv::Point(imgX, y + lineH), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                    cv::putText(debugImg, line3.str(), cv::Point(imgX, y + lineH*2), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
                    cv::putText(debugImg, line4.str(), cv::Point(imgX, y + lineH*3), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

                    std::string outDir = "predicted_entries";
                    std::error_code ec;
                    std::filesystem::create_directories(outDir, ec);
                    if (ec) {
                        std::cerr << "Warning: could not create directory '" << outDir << "': " << ec.message() << std::endl;
                    }
                    int imgIdx = debugImageIndex++;
                    std::ostringstream fname;
                    fname << outDir << "/predicted_entry_" << std::setw(4) << std::setfill('0') << imgIdx << ".png";
                    capture.saveImage(debugImg, fname.str());
                } catch (const std::exception& e) {
                    std::cerr << "Failed to save predicted-entry image: " << e.what() << std::endl;
                }
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