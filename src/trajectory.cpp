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

    Eigen::MatrixXd F(4, 4);
    F << 1, 0, dt, 0,
         0, 1, 0, dt,
         0, 0, 1, 0,
         0, 0, 0, 1;
    kalmanFilter_.setF(F);
    kalmanFilter_.predict();
    Eigen::VectorXd state = kalmanFilter_.getState();
    return cv::Point2f(state(0), state(1));
}

void TrajectoryPredictor::reset() {
    initialized_ = false;
    lastTimestamp_ = 0;
}
