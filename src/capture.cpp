#include "capture.hpp"
#include <iostream>

ImageCapture::ImageCapture(int cameraIndex) : cameraIndex_(cameraIndex) {}

ImageCapture::~ImageCapture() {
    if (cap_.isOpened()) {
        cap_.release();
    }
}
cv::Rect ImageCapture::detectTable(cv::Mat& image) {
    cv::Mat gray, blurred, edged;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
    cv::Canny(blurred, edged, 50, 150);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edged, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double maxArea = 0;
    double minAreaThreshold = (image.rows * image.cols) * 0.2; // Minimum area threshold to filter small contours
    cv::Rect tableRect;

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > maxArea && area > minAreaThreshold) {
            maxArea = area;
            tableRect = cv::boundingRect(contour);
        }
    }

    return tableRect;
}

bool ImageCapture::initialize() {
    cap_.open(cameraIndex_);
    if (!cap_.isOpened()) {
        std::cerr << "Error: Could not open camera " << cameraIndex_ << std::endl;
        return false;
    }
    // Try to load calibration data
    loadCalibration();
    return true;
}

cv::Mat ImageCapture::captureImage() {
    cv::Mat frame;
    if (cap_.isOpened()) {
        cap_ >> frame;
        cv::Rect tableRect = detectTable(frame);
        if (tableRect.area() > 0) {
            frame = frame(tableRect);
            croppedWidth_ = tableRect.width;
            croppedHeight_ = tableRect.height;
        }
    }
    return frame;
}

cv::Mat ImageCapture::captureGrayscaleImage() {
    cv::Mat frame = captureImage();
    if (!frame.empty()) {
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    }
    return frame;
}

bool ImageCapture::saveImage(const cv::Mat& image, const std::string& filename) {
    if (image.empty()) {
        std::cerr << "Error: Image is empty, cannot save." << std::endl;
        return false;
    }
    return cv::imwrite(filename, image);
}

cv::Point2f ImageCapture::detectPuck(const cv::Mat& grayImage) {
    cv::Mat thresh;
    cv::threshold(grayImage, thresh, PUCK_THRESHOLD, 255, cv::THRESH_BINARY_INV);  // Invert for black puck

    //parameters for SimpleBlobDetector
    cv::SimpleBlobDetector::Params params;
    params.filterByArea = true;
    params.minArea = PUCK_MIN_AREA; 
    params.maxArea = PUCK_MAX_AREA;
    params.filterByCircularity = true;
    params.minCircularity = 0.5f;
    params.filterByInertia = true;
    params.minInertiaRatio = 0.5f;
    params.filterByConvexity = true;
    params.minConvexity = 0.8f;
    params.filterByColor = true;
    params.blobColor = 255;  // Detect white blobs (inverted black puck)
    
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(thresh, keypoints);  // Detect on thresholded image

    if(!keypoints.empty()) {
        
        return keypoints[0].pt; // Return the position of the first detected puck
    }

    return cv::Point2f(-1, -1); // Return an invalid point if no puck is detected
}

cv::Point2f ImageCapture::imageToTableCoordinates(cv::Point2f imagePoint, int imageWidth, int imageHeight) {
    // First, undistort the point if calibration is available
    cv::Point2f undistortedPoint = undistortPoint(imagePoint);

    // Scale factors from image pixels to physical units (mm)
    float scaleX = PHYSICAL_TABLE_WIDTH / croppedWidth_;
    float scaleY = PHYSICAL_TABLE_HEIGHT / croppedHeight_;

    // Convert to table coordinates with origin at center
    // Image origin is top-left, table origin at bottom left
    float tableX = (undistortedPoint.x - croppedWidth_ / 2.0f) * scaleX;
    float tableY = (croppedHeight_ / 2.0f - undistortedPoint.y) * scaleY;  // Flip Y since image Y increases downward

    
    return cv::Point2f(tableX, tableY);
}

bool ImageCapture::loadCalibration(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cout << "Calibration file not found: " << filename << ". Using uncalibrated coordinates." << std::endl;
        return false;
    }

    fs["camera_matrix"] >> cameraMatrix_;
    fs["distortion_coefficients"] >> distCoeffs_;
    fs.release();

    std::cout << "Calibration loaded from: " << filename << std::endl;
    return true;
}

cv::Point2f ImageCapture::undistortPoint(cv::Point2f distortedPoint) {
    if (cameraMatrix_.empty() || distCoeffs_.empty()) {
        // No calibration available, return original point
        return distortedPoint;
    }

    // Undistort the point
    std::vector<cv::Point2f> points = {distortedPoint};
    cv::undistortPoints(points, points, cameraMatrix_, distCoeffs_, cv::noArray(), cameraMatrix_);

    return points[0];
}
