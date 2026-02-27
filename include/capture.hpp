#ifndef CAPTURE_HPP
#define CAPTURE_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "config.hpp"

class ImageCapture {
public:
    ImageCapture(const Config& config);
    ~ImageCapture();
    cv::RotatedRect detectTable(cv::Mat& image);
    bool initialize();
    cv::Mat captureImage();
    cv::Mat captureRawImage();
    cv::Mat captureGrayscaleImage();
    bool saveImage(const cv::Mat& image, const std::string& filename);
    cv::Point2f detectPuck(const cv::Mat& grayImage);
    cv::Point2f imageToTableCoordinates(cv::Point2f imagePoint, int imageWidth, int imageHeight);
    cv::Point2f TableToImageCoordinates(cv::Point2f tablePoint, int imageWidth, int imageHeight);
    bool loadCalibration(const std::string& filename = "calibration_result.yaml");
    cv::Point2f undistortPoint(cv::Point2f distortedPoint);
    bool saveCachedPerspective(const std::string& filename = "table_perspective.yml");
    bool loadCachedPerspective(const std::string& filename = "table_perspective.yml");
    int getCroppedWidth() const { return croppedWidth_; }
    int getCroppedHeight() const { return croppedHeight_; }
    void tableFound(bool found) { tableDetected_ = found; }
    

private:
    int croppedWidth_;
    int croppedHeight_;
    cv::VideoCapture cap_;
    int cameraIndex_;
    
    // Camera calibration data 
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    
    // Table detection and perspective correction data
    bool tablePerspectiveCached_;
    bool tableDetected_;
    cv::Mat tablePerspectiveMatrix_;
    cv::Rect tableBoundingRect_;
    cv::Size tableOutputSize_;
    
    const Config& config_;
};

#endif // CAPTURE_HPP
