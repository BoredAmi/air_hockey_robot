#include "trajectory.hpp"

TrajectoryPredictor::TrajectoryPredictor() : kalmanFilter_(), lastTimestamp_(0), initialized_(false) {}

void TrajectoryPredictor::addMeasurement(const PuckPosition& measurement) {
    if (!initialized_) {
        lastTimestamp_ = measurement.timestamp;
        Eigen::VectorXd initialState(4);
        initialState << measurement.position.x, measurement.position.y, 0, 0;
        kalmanFilter_.setState(initialState);
        initialized_ = true;
        return;
    }

    double dt = (measurement.timestamp - lastTimestamp_) / 1000000.0;  // Convert microseconds to seconds
    if (dt <= 0) return;  
    lastTimestamp_ = measurement.timestamp;

    // Update F with dt
    Eigen::MatrixXd F(4, 4);
    F << 1, 0, dt, 0,
         0, 1, 0, dt,
         0, 0, 1, 0,
         0, 0, 0, 1;
    kalmanFilter_.setF(F);

    Eigen::VectorXd meas(2);
    meas << measurement.position.x, measurement.position.y;
    kalmanFilter_.predict();
    kalmanFilter_.update(meas);
}

cv::Point2f TrajectoryPredictor::predictPosition(uint64_t futureTimestamp) {
    if (!initialized_) return cv::Point2f(-1, -1);

    double dt = (futureTimestamp - lastTimestamp_) / 1000000.0;
    if (dt < 0) return cv::Point2f(-1, -1);

    Eigen::VectorXd state = kalmanFilter_.getState();  // [x, y, vx, vy]
    double timeLeft = dt;
    cv::Point2f pos(state(0), state(1));
    double vx = state(2), vy = state(3);
    const int maxBounces = 3; 

    for (int bounce = 0; bounce < maxBounces && timeLeft > 0; ++bounce) {
        // Calculate time to hit each boundary
        double tx_left = (vx != 0) ? ((vx < 0) ? (0 - pos.x) / vx : 1e9) : 1e9;  // Hit left wall at x=0 if moving left
        double tx_right = (vx != 0) ? ((vx > 0) ? (PHYSICAL_TABLE_WIDTH - pos.x) / vx : 1e9) : 1e9;  // Hit right wall at x=1000 if moving right
        double ty_bottom = (vy != 0) ? ((vy < 0) ? (0 - pos.y) / vy : 1e9) : 1e9;  // Hit bottom wall at y=0 if moving down


        // Find the earliest hit time within remaining time
        double t_hit = std::min({tx_left, tx_right, ty_bottom, timeLeft});
        if (t_hit < 0 || t_hit > timeLeft) t_hit = timeLeft;  // No valid hit, just advance

        // Move to hit point or end time
        pos.x += vx * t_hit;
        pos.y += vy * t_hit;
        timeLeft -= t_hit;

        if (timeLeft <= 0) break;  // Reached target time

        // Reflect velocity at boundary
        if (t_hit == tx_left || t_hit == tx_right) vx = -vx;
        if (t_hit == ty_bottom) vy = -vy;
    }


    // Clamp to bounds if still out (rare)
    if (pos.x < 0) pos.x = 0;
    if (pos.x > PHYSICAL_TABLE_WIDTH) pos.x = PHYSICAL_TABLE_WIDTH;
    if (pos.y < 0) pos.y = 0;
    if (pos.y > PHYSICAL_TABLE_HEIGHT) pos.y = PHYSICAL_TABLE_HEIGHT;

    return pos;
}

void TrajectoryPredictor::reset() {
    initialized_ = false;
    lastTimestamp_ = 0;
}
