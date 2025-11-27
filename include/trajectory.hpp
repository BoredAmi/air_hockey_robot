#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP
#include <opencv2/opencv.hpp>
#include <vector>
#include "kalman.hpp"
#include "config.hpp"

struct PuckPosition {
    cv::Point2f position;  // mm
    uint64_t timestamp;    
};

class TrajectoryPredictor {
public:
    TrajectoryPredictor();
    void addMeasurement(const PuckPosition& measurement);
    cv::Point2f predictPosition(uint64_t futureTimestamp);
    cv::Point2f predictEntryToDefenseZone(uint64_t currentTimestamp);
    void reset();  // Reset for lost puck
private:
    KalmanFilter kalmanFilter_;
    uint64_t lastTimestamp_;
    bool initialized_;
};
#endif // TRAJECTORY_HPP