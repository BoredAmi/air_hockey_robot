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
    cv::Rect detectTable(cv::Mat& image);
    bool initialize();
    cv::Mat captureImage();
    cv::Mat captureGrayscaleImage();
    bool saveImage(const cv::Mat& image, const std::string& filename);
    cv::Point2f detectPuck(const cv::Mat& grayImage);
    cv::Point2f imageToTableCoordinates(cv::Point2f imagePoint, int imageWidth, int imageHeight);
    cv::Point2f TableToImageCoordinates(cv::Point2f tablePoint, int imageWidth, int imageHeight);
    bool loadCalibration(const std::string& filename = "camera_calibration.yml");
    cv::Point2f undistortPoint(cv::Point2f distortedPoint);
    int getCroppedWidth() const { return croppedWidth_; }
    int getCroppedHeight() const { return croppedHeight_; }
    

private:
    int croppedWidth_;
    int croppedHeight_;
    cv::VideoCapture cap_;
    int cameraIndex_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
};

#endif // CAPTURE_HPP
