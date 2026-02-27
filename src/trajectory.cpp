#include "trajectory.hpp"

TrajectoryPredictor::TrajectoryPredictor(const Config& config) : config_(config), currentZoneIndex_(config.WHERE_DEFENSE_ZONE), kalmanFilter_(), lastTimestamp_(0), initialized_(false) {
        // Defense zone bounds
    setDefenseZone(config.WHERE_DEFENSE_ZONE);
}

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
        double tx_left = (vx != 0) ? ((vx < 0) ? (0 - pos.x) / vx : 1e9) : 1e9;  // Hit left wall if moving left
        double tx_right = (vx != 0) ? ((vx > 0) ? (config_.PHYSICAL_TABLE_WIDTH - pos.x) / vx : 1e9) : 1e9;  // Hit right wall if moving right
        double ty_bottom = (vy != 0) ? ((vy < 0) ? (0 - pos.y) / vy : 1e9) : 1e9;  // Hit bottom wall if moving down
        double ty_top = (vy != 0) ? ((vy > 0) ? (config_.PHYSICAL_TABLE_HEIGHT - pos.y) / vy : 1e9) : 1e9;  // Hit top wall if moving up


        // Find the earliest hit time within remaining time
        double t_hit = std::min({tx_left, tx_right, ty_bottom, timeLeft});
        if (t_hit < 0 || t_hit > timeLeft) t_hit = timeLeft;  // No valid hit, just advance

        // Move to hit point or end time
        pos.x += vx * t_hit;
        pos.y += vy * t_hit;
        timeLeft -= t_hit;

        if (timeLeft <= 0) break;  // Reached target time
        switch (currentZoneIndex_) //prevent bouncing at the oposite side to the defense zone
        {
        case 0: // Left defense zone reflect velocity at boundary except for right wall
                if (t_hit == tx_left) vx = -vx;
                if (t_hit == ty_bottom || t_hit == ty_top) vy = -vy;
                break;
            break;
        case 1: // Right defense zone
                if (t_hit == tx_right) vx = -vx;
                if (t_hit == ty_bottom || t_hit == ty_top) vy = -vy;
            break;
        case 2: // Bottom defense zone
                if (t_hit == ty_bottom) vy = -vy;
                if (t_hit == tx_left || t_hit == tx_right) vx = -vx;
            break;
        case 3: // Top defense zone
                if (t_hit == ty_top) vy = -vy;
                if (t_hit == tx_left || t_hit == tx_right) vx = -vx;    

            break;
        
        default:
            break;
        }
    }


    // Clamp to bounds if still out (rare)
    if (pos.x < 0) pos.x = 0;
    if (pos.x > config_.PHYSICAL_TABLE_WIDTH) pos.x = config_.PHYSICAL_TABLE_WIDTH;
    if (pos.y < 0) pos.y = 0;
    if (pos.y > config_.PHYSICAL_TABLE_HEIGHT) pos.y = config_.PHYSICAL_TABLE_HEIGHT;

    return pos;
}
cv::Point2f TrajectoryPredictor::predictEntryToDefenseZone(uint64_t currentTimestamp) {
    if (!initialized_) return cv::Point2f(-1, -1);

    Eigen::VectorXd state = kalmanFilter_.getState();  // [x, y, vx, vy]
    cv::Point2f pos(state(0), state(1));
    double vx = state(2), vy = state(3);
    double timeAccum = 0.0;
    const int maxBounces = 5;
    const double maxTime = 2.0; // 1 second
    const double dt = 0.01; // Small time step for simulation (10ms)
    bool didnthitboundary = true;


    // If already in zone, return current position
    if (pos.y <= zoneYMax && pos.y >= zoneYMin && pos.x >= zoneXMin && pos.x <= zoneXMax) {
        return pos;
    }

    for (int bounce = 0; bounce < maxBounces && timeAccum < maxTime; ++bounce) {
        // Simulate in small steps until zone entry or boundary hit
        while (timeAccum < maxTime) {
            // Check if in zone after this step
            cv::Point2f nextPos = pos;
            nextPos.x += vx * dt;
            nextPos.y += vy * dt;

            if (nextPos.y <= zoneYMax && nextPos.y >= zoneYMin && nextPos.x >= zoneXMin && nextPos.x <= zoneXMax) {
                // Entered zone - interpolate exact entry point
                // (Simple linear interp; could be more precise)
                double entryX = pos.x + vx * (dt / 2.0); // Approximate
                double entryY = pos.y + vy * (dt / 2.0);
                return cv::Point2f(entryX, entryY);
            }
            // Check for boundary hits and reflect
            switch (currentZoneIndex_) //prevent bouncing at the oposite side to the defense zone
            {
            case 0: // Left defense zone reflect velocity at boundary except for right wall
                    if (nextPos.x <= 0 ) vx = -vx;
                    if (nextPos.y <= 0 || nextPos.y >= config_.PHYSICAL_TABLE_HEIGHT) vy = -vy;
                break;
            case 1: // Right defense zone
                    if (nextPos.x >= config_.PHYSICAL_TABLE_WIDTH) vx = -vx;
                    if (nextPos.y <= 0 || nextPos.y >= config_.PHYSICAL_TABLE_HEIGHT) vy = -vy;
                break;
            case 2: // Bottom defense zone
                    if (nextPos.y >= config_.PHYSICAL_TABLE_HEIGHT) vy = -vy;
                    if (nextPos.x <= 0 || nextPos.x >= config_.PHYSICAL_TABLE_WIDTH) vx = -vx;
                break;
            case 3: // Top defense zone
                    if (nextPos.y <= 0) vy = -vy;
                    if (nextPos.x <= 0 || nextPos.x >= config_.PHYSICAL_TABLE_WIDTH) vx = -vx;    
                break;  
            default:
                    didnthitboundary = true;
                break;
            }

            pos = nextPos;
            timeAccum += dt;

            // If bounced, break inner loop to count bounce
            if (!didnthitboundary) {
                didnthitboundary = true;
                break;
            }
        }
    }

    return cv::Point2f(-1, -1);  // No entry within limits
}
void TrajectoryPredictor::reset() {
    initialized_ = false;
    lastTimestamp_ = 0;
}
bool TrajectoryPredictor::isInDefenseZone(const cv::Point2f& pos) {
    return (pos.y <= zoneYMax && pos.y >= zoneYMin && pos.x >= zoneXMin && pos.x <= zoneXMax);
}
void TrajectoryPredictor::setDefenseZone(int zoneIndex) {
    currentZoneIndex_ = zoneIndex;
    switch (currentZoneIndex_) {
    case 0: // Left defense zone
        zoneYMin = (config_.PHYSICAL_TABLE_HEIGHT - config_.DEFENSE_ZONE_WIDTH) / 2.0;
        zoneYMax = (config_.PHYSICAL_TABLE_HEIGHT + config_.DEFENSE_ZONE_WIDTH) / 2.0;
        zoneXMin = 0;
        zoneXMax = config_.DEFENSE_ZONE_HEIGHT;
        break;
    case 1: // Right defense zone
        zoneYMin = (config_.PHYSICAL_TABLE_HEIGHT - config_.DEFENSE_ZONE_WIDTH) / 2.0;
        zoneYMax = (config_.PHYSICAL_TABLE_HEIGHT + config_.DEFENSE_ZONE_WIDTH) / 2.0;
        zoneXMin = config_.PHYSICAL_TABLE_WIDTH - config_.DEFENSE_ZONE_HEIGHT;
        zoneXMax = config_.PHYSICAL_TABLE_WIDTH;
        break;
    case 2: // Top defense zone
        zoneYMin = 0;
        zoneYMax = config_.DEFENSE_ZONE_HEIGHT;
        zoneXMin = (config_.PHYSICAL_TABLE_WIDTH - config_.DEFENSE_ZONE_WIDTH) / 2.0;
        zoneXMax = (config_.PHYSICAL_TABLE_WIDTH + config_.DEFENSE_ZONE_WIDTH) / 2.0;

        break;
    case 3: // Bottom defense zone
        zoneYMin = config_.PHYSICAL_TABLE_HEIGHT - config_.DEFENSE_ZONE_HEIGHT;
        zoneYMax = config_.PHYSICAL_TABLE_HEIGHT;
        zoneXMin = (config_.PHYSICAL_TABLE_WIDTH - config_.DEFENSE_ZONE_WIDTH) / 2.0;
        zoneXMax = (config_.PHYSICAL_TABLE_WIDTH + config_.DEFENSE_ZONE_WIDTH) / 2.0;
        break;
    
    default:
        break;
    }
}
