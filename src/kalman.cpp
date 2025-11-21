#include "kalman.hpp"

KalmanFilter::KalmanFilter() {
    // State: [x, y, vx, vy]
    state_ = Eigen::VectorXd(4);
    state_ << 0, 0, 0, 0;  // Initial state

    // Covariance matrix - high initial uncertainty
    P_ = Eigen::MatrixXd::Identity(4, 4) * 1000;

    // State transition matrix 
    F_ = Eigen::MatrixXd::Identity(4, 4);

    // Measurement matrix 
    H_ = Eigen::MatrixXd(2, 4);
    H_ << 1, 0, 0, 0,
          0, 1, 0, 0;

    // Process noise 
    Q_ = Eigen::MatrixXd::Identity(4, 4) * 0.01;

    // Measurement noise
    R_ = Eigen::MatrixXd::Identity(2, 2) * 0.1;
}

void KalmanFilter::predict() {
    state_ = F_ * state_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd& measurement) {
    Eigen::VectorXd y = measurement - H_ * state_;
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    state_ = state_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(4, 4) - K * H_) * P_;
}

Eigen::VectorXd KalmanFilter::getState() const {
    return state_;
}

void KalmanFilter::setState(const Eigen::VectorXd& state) {
    state_ = state;
}

void KalmanFilter::setF(const Eigen::MatrixXd& F) {
    F_ = F;
}
