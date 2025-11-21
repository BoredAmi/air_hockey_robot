#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP
#include <opencv2/opencv.hpp>
#include <vector>
struct PuckPosition {
    cv::Point2f position;  // mm
    uint64_t timestamp;    
};
// Class to predict puck trajectory using kalman filter
class TrajectoryPredictor {
public:
    TrajectoryPredictor();
    void addMeasurement(const PuckPosition& measurement);
    cv::Point2f predictPosition(uint64_t futureTimestamp);
};
#endif // TRAJECTORY_HPP