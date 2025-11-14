#include "trajectory.hpp"
void TrajectoryPredictor::reset() {
    positions_.clear();
}
void TrajectoryPredictor::addPosition(const PuckPosition& pos) {
    positions_.push_back(pos);
    if (positions_.size() > MAX_POSITION_HISTORY) {
        positions_.erase(positions_.begin());
    }
}
cv::Point2f TrajectoryPredictor::computeVelocity() {
    if (positions_.size() < 2) {
        return cv::Point2f(0.0f, 0.0f);
    }
    const PuckPosition& last = positions_.back();
    const PuckPosition& prev = positions_[positions_.size() - 2];
    float dt = static_cast<float>(last.timestamp - prev.timestamp);
    if (dt <= 0.0f) {
        return cv::Point2f(0.0f, 0.0f);
    }
    float vx = (last.position.x - prev.position.x) / dt;
    float vy = (last.position.y - prev.position.y) / dt;
    return cv::Point2f(vx, vy);
}
cv::Point2f TrajectoryPredictor::predictPosition(uint64_t futureTime) {
    if (positions_.empty()) {
        return cv::Point2f(0.0f, 0.0f);
    }
    const PuckPosition& last = positions_.back();
    cv::Point2f velocity = computeVelocity();
    float dt = static_cast<float>(futureTime - last.timestamp);
    float predictedX = last.position.x + velocity.x * dt;
    float predictedY = last.position.y + velocity.y * dt;
    return cv::Point2f(predictedX, predictedY);
}