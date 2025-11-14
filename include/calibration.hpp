#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "config.hpp"

class CameraCalibration {
public:
    CameraCalibration(int cameraIndex = CAMERA_INDEX);
    ~CameraCalibration();

    bool initialize();
    bool captureCalibrationImage();
    bool calibrateCamera();
    bool saveCalibration(const std::string& filename);
    bool loadCalibration(const std::string& filename);
    cv::Mat getCameraMatrix() const { return cameraMatrix_; }
    cv::Mat getDistCoeffs() const { return distCoeffs_; }
    void undistortImage(const cv::Mat& input, cv::Mat& output);

private:
    cv::VideoCapture cap_;
    int cameraIndex_;
    std::vector<std::vector<cv::Point2f>> imagePoints_;
    std::vector<std::vector<cv::Point3f>> objectPoints_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    cv::Size boardSize_;  // Chessboard size (corners per row/column)
    float squareSize_;    // Size of chessboard squares in mm

    std::vector<cv::Point3f> createObjectPoints();
};

#endif // CALIBRATION_HPP
