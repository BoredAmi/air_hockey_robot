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
cv::Point2f TrajectoryPredictor::predictEntryToDefenseZone(uint64_t currentTimestamp) {
    if (!initialized_) return cv::Point2f(-1, -1);

    Eigen::VectorXd state = kalmanFilter_.getState();  // [x, y, vx, vy]
    cv::Point2f pos(state(0), state(1));
    double vx = state(2), vy = state(3);
    double timeAccum = 0.0;
    const int maxBounces = 5;
    const double maxTime = 1.0; // 1 second

    // Defense zone: Y from 0 to DEFENSE_ZONE_Y (100mm), X centered with width DEFENSE_ZONE_X (300mm)
    const double zoneYMax = DEFENSE_ZONE_Y;  // 100mm
    const double zoneXMin = (PHYSICAL_TABLE_WIDTH - DEFENSE_ZONE_X) / 2.0;  // (1000 - 300)/2 = 350
    const double zoneXMax = (PHYSICAL_TABLE_WIDTH + DEFENSE_ZONE_X) / 2.0;  // (1000 + 300)/2 = 650

    // If already in zone, return current position
    if (pos.y <= zoneYMax && pos.x >= zoneXMin && pos.x <= zoneXMax) {
        return pos;
    }

    for (int bounce = 0; bounce < maxBounces && timeAccum < maxTime; ++bounce) {
        // Calculate times to boundaries
        double tx_left = (vx < 0) ? (0 - pos.x) / vx : 1e9;
        double tx_right = (vx > 0) ? (PHYSICAL_TABLE_WIDTH - pos.x) / vx : 1e9;
        double ty_bottom = (vy < 0) ? (0 - pos.y) / vy : 1e9;
        double ty_top = (vy > 0) ? (PHYSICAL_TABLE_HEIGHT - pos.y) / vy : 1e9;

        // Time to enter defense zone
        double t_enter_y = (vy < 0 && pos.y > zoneYMax) ? (zoneYMax - pos.y) / vy : 1e9;  // Entering from top (y direction)
        double t_enter_x_min = (vx > 0 && pos.x < zoneXMin) ? (zoneXMin - pos.x) / vx : 1e9;  // Entering from left (x direction)
        double t_enter_x_max = (vx < 0 && pos.x > zoneXMax) ? (zoneXMax - pos.x) / vx : 1e9;  // Entering from right (x direction)
        double t_enter = std::min({t_enter_y, t_enter_x_min, t_enter_x_max});

        // Find earliest event
        double t_hit = std::min({tx_left, tx_right, ty_bottom, ty_top, t_enter});
        double t_step = std::min(t_hit, maxTime - timeAccum);
        if (t_step <= 0) break;

        // Move
        pos.x += vx * t_step;
        pos.y += vy * t_step;
        timeAccum += t_step;

        // If entered zone, return position
        if (t_hit == t_enter) {
            return pos;
        }

        // Reflect if hit boundary
        if (t_hit == tx_left || t_hit == tx_right) vx = -vx;
        if (t_hit == ty_bottom || t_hit == ty_top) vy = -vy;
    }

    return cv::Point2f(-1, -1);  // No entry within maxTime
}
void TrajectoryPredictor::reset() {
    initialized_ = false;
    lastTimestamp_ = 0;
}
