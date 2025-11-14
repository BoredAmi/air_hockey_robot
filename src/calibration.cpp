#include "calibration.hpp"
#include <iostream>
#include <fstream>

CameraCalibration::CameraCalibration(int cameraIndex)
    : cameraIndex_(cameraIndex), boardSize_(CHESSBOARD_WIDTH, CHESSBOARD_HEIGHT), squareSize_(SQUARE_SIZE) {}

CameraCalibration::~CameraCalibration() {
    if (cap_.isOpened()) {
        cap_.release();
    }
}

bool CameraCalibration::initialize() {
    cap_.open(cameraIndex_);
    if (!cap_.isOpened()) {
        std::cerr << "Error: Could not open camera " << cameraIndex_ << std::endl;
        return false;
    }
    return true;
}

bool CameraCalibration::captureCalibrationImage() {
    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty()) {
        std::cerr << "Error: Could not capture frame" << std::endl;
        return false;
    }

    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(frame, boardSize_, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if (found) {
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

        imagePoints_.push_back(corners);
        objectPoints_.push_back(createObjectPoints());

        std::cout << "Calibration image captured. Total images: " << imagePoints_.size() << std::endl;
        return true;
    } else {
        std::cout << "Chessboard not found in image. Try adjusting position." << std::endl;
        return false;
    }
}

std::vector<cv::Point3f> CameraCalibration::createObjectPoints() {
    std::vector<cv::Point3f> objPoints;
    for (int i = 0; i < boardSize_.height; i++) {
        for (int j = 0; j < boardSize_.width; j++) {
            objPoints.push_back(cv::Point3f(j * squareSize_, i * squareSize_, 0));
        }
    }
    return objPoints;
}

bool CameraCalibration::calibrateCamera() {
    if (imagePoints_.size() < 10) {
        std::cerr << "Error: Need at least 10 calibration images. Currently have: " << imagePoints_.size() << std::endl;
        return false;
    }

    cv::Size imageSize;
    cv::Mat frame;
    cap_ >> frame;
    imageSize = frame.size();

    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(objectPoints_, imagePoints_, imageSize, cameraMatrix_, distCoeffs_, rvecs, tvecs);

    std::cout << "Calibration completed. RMS error: " << rms << std::endl;
    return true;
}

bool CameraCalibration::saveCalibration(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "Error: Could not open file for writing: " << filename << std::endl;
        return false;
    }

    fs << "camera_matrix" << cameraMatrix_;
    fs << "distortion_coefficients" << distCoeffs_;
    fs.release();

    std::cout << "Calibration saved to: " << filename << std::endl;
    return true;
}

bool CameraCalibration::loadCalibration(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Error: Could not open file for reading: " << filename << std::endl;
        return false;
    }

    fs["camera_matrix"] >> cameraMatrix_;
    fs["distortion_coefficients"] >> distCoeffs_;
    fs.release();

    std::cout << "Calibration loaded from: " << filename << std::endl;
    return true;
}

void CameraCalibration::undistortImage(const cv::Mat& input, cv::Mat& output) {
    cv::undistort(input, output, cameraMatrix_, distCoeffs_);
}
