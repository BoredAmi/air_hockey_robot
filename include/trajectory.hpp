#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP
#include <opencv2/opencv.hpp>
#include <vector>
struct PuckPosition {
    cv::Point2f position;  // mm
    uint64_t timestamp;    
};
class TrajectoryPredictor {
public:
    int MAX_POSITION_HISTORY = 10;
    void reset();
    void addPosition(const PuckPosition& pos);
    cv::Point2f predictPosition(uint64_t futureTime);
private:
    std::vector<PuckPosition> positions_;
    cv::Point2f computeVelocity();
};
#endif // TRAJECTORY_HPP