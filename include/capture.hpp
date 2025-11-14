#ifndef CAPTURE_HPP
#define CAPTURE_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "config.hpp"

class ImageCapture {
public:
    ImageCapture(int cameraIndex = CAMERA_INDEX);
    ~ImageCapture();

    bool initialize();
    cv::Mat captureImage();
    cv::Mat captureGrayscaleImage();
    bool saveImage(const cv::Mat& image, const std::string& filename);
    std::vector<cv::Vec3f> detectPuck(const cv::Mat& grayImage);
    cv::Point2f imageToTableCoordinates(cv::Point2f imagePoint, int imageWidth, int imageHeight);
    bool loadCalibration(const std::string& filename = "camera_calibration.yml");
    cv::Point2f undistortPoint(cv::Point2f distortedPoint);

private:
    cv::VideoCapture cap_;
    int cameraIndex_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
};

#endif // CAPTURE_HPP
