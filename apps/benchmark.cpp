#include "capture.hpp"
#include "trajectory.hpp"
#include "movement.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <iostream>
#include <string>
#include <numeric>

int main() {
    Config config;
    config.loadFromFile();
    cv::setUseOptimized(true);
    ImageCapture capture(config);
    if (!capture.initialize()) {
        std::cerr << "Failed to initialize camera." << std::endl;
        return -1;
    }

    // Assume table is found for benchmark
    capture.tableFound(true);

    TrajectoryPredictor predictor(config);
    MovementController mover(config);

    std::cout << "Starting air hockey robot benchmark..." << std::endl;
    std::cout << "Running for 60 seconds without GUI..." << std::endl;

    // Benchmark parameters
    const int benchmarkDurationSeconds = 60;
    auto startTime = std::chrono::high_resolution_clock::now();
    auto endTime = startTime + std::chrono::seconds(benchmarkDurationSeconds);

    // Statistics
    int totalFrames = 0;
    int framesWithPuckDetected = 0;
    int framesWithPrediction = 0;
    int movementsMade = 0;

    // Timing vectors for averages
    std::vector<double> captureTimes;
    std::vector<double> detectionTimes;
    std::vector<double> predictionTimes;
    std::vector<double> movementTimes;

    while (std::chrono::high_resolution_clock::now() < endTime) {
        auto frameStart = std::chrono::high_resolution_clock::now();

        // Capture frame
        auto captureStart = std::chrono::high_resolution_clock::now();
        cv::Mat frame = capture.captureImage();
        auto captureEnd = std::chrono::high_resolution_clock::now();
        double captureTime = std::chrono::duration<double, std::milli>(captureEnd - captureStart).count();
        captureTimes.push_back(captureTime);

        if (frame.empty()) {
            std::cerr << "Failed to capture frame." << std::endl;
            continue;
        }

        totalFrames++;

        // Convert to gray
        auto detectionStart = std::chrono::high_resolution_clock::now();
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Detect puck
        cv::Point2f puckCenter = capture.detectPuck(gray);
        bool puckDetected = (puckCenter.x >= 0 && puckCenter.y >= 0);
        auto detectionEnd = std::chrono::high_resolution_clock::now();
        double detectionTime = std::chrono::duration<double, std::milli>(detectionEnd - detectionStart).count();
        detectionTimes.push_back(detectionTime);

        if (puckDetected) {
            framesWithPuckDetected++;
        }

        double currentTime = cv::getTickCount() / cv::getTickFrequency();
        uint64_t currentTimeUs = (uint64_t)(currentTime * 1000000.0);

        cv::Point2f predictedEntryTable;
        predictedEntryTable.x = -1.0f;
        predictedEntryTable.y = -1.0f;

        if (puckDetected) {
            PuckPosition puckPos = {capture.imageToTableCoordinates(puckCenter, capture.getCroppedWidth(), capture.getCroppedHeight()), currentTimeUs};
            predictor.addMeasurement(puckPos);

            // Only predict if puck is outside defense zone
            if (!predictor.isInDefenseZone(puckPos.position)) {
                auto predictionStart = std::chrono::high_resolution_clock::now();
                // Predict entry to defense zone
                predictedEntryTable = predictor.predictEntryToDefenseZone(currentTimeUs);
                auto predictionEnd = std::chrono::high_resolution_clock::now();
                double predictionTime = std::chrono::duration<double, std::milli>(predictionEnd - predictionStart).count();
                predictionTimes.push_back(predictionTime);
                framesWithPrediction++;
            }
        }

        if (predictedEntryTable.x >= 0 && predictedEntryTable.y >= 0) {
            // Move robot
            auto movementStart = std::chrono::high_resolution_clock::now();
            cv::Point2f robotPosInTable(predictedEntryTable.x, predictedEntryTable.y);
            cv::Point2f robotPos = mover.TableToRobotCoordinates(robotPosInTable);
            mover.moveTo(robotPos);
            auto movementEnd = std::chrono::high_resolution_clock::now();
            double movementTime = std::chrono::duration<double, std::milli>(movementEnd - movementStart).count();
            movementTimes.push_back(movementTime);
            movementsMade++;
        }

        auto frameEnd = std::chrono::high_resolution_clock::now();
        double frameTime = std::chrono::duration<double, std::milli>(frameEnd - frameStart).count();

        // Optional: print progress every second
        static auto lastPrint = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - lastPrint).count() >= 1.0) {
            std::cout << "Processed " << totalFrames << " frames so far..." << std::endl;
            lastPrint = std::chrono::high_resolution_clock::now();
        }
    }

    // Calculate statistics
    double totalTime = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - startTime).count();
    double avgFps = totalFrames / totalTime;

    double avgCaptureTime = captureTimes.empty() ? 0 : std::accumulate(captureTimes.begin(), captureTimes.end(), 0.0) / captureTimes.size();
    double avgDetectionTime = detectionTimes.empty() ? 0 : std::accumulate(detectionTimes.begin(), detectionTimes.end(), 0.0) / detectionTimes.size();
    double avgPredictionTime = predictionTimes.empty() ? 0 : std::accumulate(predictionTimes.begin(), predictionTimes.end(), 0.0) / predictionTimes.size();
    double avgMovementTime = movementTimes.empty() ? 0 : std::accumulate(movementTimes.begin(), movementTimes.end(), 0.0) / movementTimes.size();

    double detectionRate = totalFrames > 0 ? (double)framesWithPuckDetected / totalFrames * 100.0 : 0;
    double predictionRate = framesWithPuckDetected > 0 ? (double)framesWithPrediction / framesWithPuckDetected * 100.0 : 0;

    // Output results
    std::cout << "\n=== Benchmark Results ===" << std::endl;
    std::cout << "Total frames processed: " << totalFrames << std::endl;
    std::cout << "Average FPS: " << avgFps << std::endl;
    std::cout << "Frames with puck detected: " << framesWithPuckDetected << " (" << detectionRate << "%)" << std::endl;
    std::cout << "Frames with prediction: " << framesWithPrediction << " (" << predictionRate << "% of detected)" << std::endl;
    std::cout << "Movements made: " << movementsMade << std::endl;
    std::cout << "\nAverage times (ms):" << std::endl;
    std::cout << "  Capture: " << avgCaptureTime << std::endl;
    std::cout << "  Detection: " << avgDetectionTime << std::endl;
    if (!predictionTimes.empty()) {
        std::cout << "  Prediction: " << avgPredictionTime << std::endl;
    }
    if (!movementTimes.empty()) {
        std::cout << "  Movement: " << avgMovementTime << std::endl;
    }

    mover.stop();
    std::cout << "Benchmark completed." << std::endl;
    return 0;
}
