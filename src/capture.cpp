#include "capture.hpp"
#include <iostream>

ImageCapture::ImageCapture(const Config& config) : config_(config), cameraIndex_(config.CAMERA_INDEX), croppedWidth_(config.TABLE_WIDTH), croppedHeight_(config.TABLE_HEIGHT), tableDetected_(false), tablePerspectiveCached_(false) {}

ImageCapture::~ImageCapture() {
    if (cap_.isOpened()) {
        cap_.release();
    }
}
cv::RotatedRect ImageCapture::detectTable(cv::Mat& image) {
    cv::Mat gray, thresh, morphed;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    
    cv::threshold(gray, thresh, 150, 255, cv::THRESH_BINARY_INV);  // Invert to get black as white
    
    // Morphological closing to connect outline segments
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(thresh, morphed, cv::MORPH_CLOSE, kernel);
    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(morphed, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat contourImg = image.clone();
    cv::drawContours(contourImg, contours, -1, cv::Scalar(0, 255, 0), 2);

    double maxArea = 0;
    double minAreaThreshold = (image.rows * image.cols) * 0.1;  
    cv::RotatedRect tableRotated;

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > maxArea && area > minAreaThreshold) {
            // Use minAreaRect directly on the contour for better handling of rotated shapes
            cv::RotatedRect minRect = cv::minAreaRect(contour);
            cv::Rect candidateRect = minRect.boundingRect();
            // Ensure the rect is within image bounds and doesn't touch border too much
            if (candidateRect.x > 0 && candidateRect.y > 0 && 
                candidateRect.x + candidateRect.width < image.cols && 
                candidateRect.y + candidateRect.height < image.rows) {
                double candidateArea = candidateRect.area();
                if (candidateArea > maxArea) {
                    maxArea = candidateArea;
                    tableRotated = minRect;
                }
            }
        }
    }

    return tableRotated;
}

bool ImageCapture::initialize() {
    if (config_.USE_LIBCAMERA_BOOL) {
        std::string pipeline = "libcamerasrc ! video/x-raw,format=BGR,width=1640,height=1232 ! appsink sync=false";
        cap_.open(pipeline, cv::CAP_GSTREAMER);
        if (!cap_.isOpened()) {
            std::cerr << "Error: Could not open camera with GStreamer pipeline" << std::endl;
            return false;
        }
    } else {
        cap_.open(config_.CAMERA_INDEX);
        if (!cap_.isOpened()) {
            std::cerr << "Error: Could not open camera " << config_.CAMERA_INDEX << std::endl;
            return false;
        }
    }
    // Try to load calibration data
    loadCalibration();
    // Try to load cached perspective data
    loadCachedPerspective();

    
    return true;
}

cv::Mat ImageCapture::captureImage() {
    cv::Mat frame;
    if (cap_.isOpened()) {
        cap_ >> frame;
        
        // Undistort the frame if calibration is available
        if (!cameraMatrix_.empty() && !distCoeffs_.empty()) {
            cv::undistort(frame, frame, cameraMatrix_, distCoeffs_);
        }
        
        cv::RotatedRect tableRotated;
        cv::Rect tableRect;
        cv::Mat perspectiveMatrix;
        cv::Size outputSize;
        if (!tablePerspectiveCached_ || !tableDetected_) {
            tableRotated = detectTable(frame);
            tableRect = tableRotated.boundingRect();
            if (tableRect.area() > 0) {
                // Cache the values
                tableBoundingRect_ = tableRect;
                tableOutputSize_ = cv::Size(tableRotated.size.width, tableRotated.size.height);
                // Compute perspective matrix
                cv::Point2f srcPoints[4];
                tableRotated.points(srcPoints);
                for (int i = 0; i < 4; i++) {
                    srcPoints[i] -= cv::Point2f(tableRect.x, tableRect.y);
                }
                cv::Point2f dstPoints[4] = {
                    {0, 0},
                    {tableRotated.size.width, 0},
                    {tableRotated.size.width, tableRotated.size.height},
                    {0, tableRotated.size.height}
                };
                tablePerspectiveMatrix_ = cv::getPerspectiveTransform(srcPoints, dstPoints);
                if (tableDetected_){
                    tablePerspectiveCached_ = true;
                    saveCachedPerspective();
                }
            }
        } else {
            tableRect = tableBoundingRect_;
            perspectiveMatrix = tablePerspectiveMatrix_;
            outputSize = tableOutputSize_;
        }
        if (tableRect.area() > 0) {
            // Crop to bounding rect
            cv::Mat cropped = frame(tableRect);
            // Warp to straighten the table
            cv::warpPerspective(cropped, frame, tablePerspectiveMatrix_, tableOutputSize_);
            cv::resize(frame, frame, cv::Size(256, 192));  // Resize for faster processing
            croppedWidth_ = 256;
            croppedHeight_ = 192;
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
    cv::threshold(grayImage, thresh, config_.PUCK_THRESHOLD, 255, cv::THRESH_BINARY_INV);  // Invert for black puck

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < config_.PUCK_MIN_AREA || area > config_.PUCK_MAX_AREA) continue;

        double perimeter = cv::arcLength(contour, true);
        double circularity = 4 * CV_PI * area / (perimeter * perimeter);
        if (circularity < 0.5) continue;  // Not circular enough

        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);
        return center;
    }

    return cv::Point2f(-1, -1);
}

cv::Point2f ImageCapture::imageToTableCoordinates(cv::Point2f imagePoint, int imageWidth, int imageHeight) {
    if (imageWidth == 0) imageWidth = config_.TABLE_WIDTH;
    if (imageHeight == 0) imageHeight = config_.TABLE_HEIGHT;

    // Scale factors from image pixels to physical units (mm)
    float scaleX = config_.PHYSICAL_TABLE_WIDTH / imageWidth;
    float scaleY = config_.PHYSICAL_TABLE_HEIGHT / imageHeight;

    // Origin at bottom-left corner of table
    float tableX = imagePoint.x * scaleX;
    float tableY = (imageHeight - imagePoint.y) * scaleY;  // Y increases up in table

    
    return cv::Point2f(tableX, tableY);
}
cv::Point2f ImageCapture::TableToImageCoordinates(cv::Point2f tablePoint, int imageWidth, int imageHeight) {
    if (imageWidth == 0) imageWidth = config_.TABLE_WIDTH;
    if (imageHeight == 0) imageHeight = config_.TABLE_HEIGHT;
    // Scale factors from physical units (mm) to image pixels
    float scaleX = imageWidth / config_.PHYSICAL_TABLE_WIDTH;
    float scaleY = imageHeight / config_.PHYSICAL_TABLE_HEIGHT;

    // Convert table coordinates back to image coordinates (origin at bottom-left)
    float imageX = tablePoint.x * scaleX;
    float imageY = imageHeight - tablePoint.y * scaleY;  // Y flip back

    cv::Point2f imagePoint(imageX, imageY);


    return imagePoint;
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

bool ImageCapture::saveCachedPerspective(const std::string& filename) {
    if (!tablePerspectiveCached_ || tablePerspectiveMatrix_.empty()) {
        std::cout << "No cached table perspective to save." << std::endl;
        return false;
    }

    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "Error: Could not open file for writing: " << filename << std::endl;
        return false;
    }

    fs << "table_rect_x" << tableBoundingRect_.x;
    fs << "table_rect_y" << tableBoundingRect_.y;
    fs << "table_rect_width" << tableBoundingRect_.width;
    fs << "table_rect_height" << tableBoundingRect_.height;
    fs << "output_width" << tableOutputSize_.width;
    fs << "output_height" << tableOutputSize_.height;
    fs << "perspective_matrix" << tablePerspectiveMatrix_;
    fs.release();

    std::cout << "Cached table perspective saved to: " << filename << std::endl;
    return true;
}

bool ImageCapture::loadCachedPerspective(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cout << "Cached table perspective file not found: " << filename << ". Will detect table automatically." << std::endl;
        return false;
    }

    int rectX, rectY, rectWidth, rectHeight, outWidth, outHeight;
    fs["table_rect_x"] >> rectX;
    fs["table_rect_y"] >> rectY;
    fs["table_rect_width"] >> rectWidth;
    fs["table_rect_height"] >> rectHeight;
    fs["output_width"] >> outWidth;
    fs["output_height"] >> outHeight;
    fs["perspective_matrix"] >> tablePerspectiveMatrix_;
    fs.release();

    tableBoundingRect_ = cv::Rect(rectX, rectY, rectWidth, rectHeight);
    tableOutputSize_ = cv::Size(outWidth, outHeight);
    tablePerspectiveCached_ = true;
    tableDetected_ = true;

    std::cout << "Cached table perspective loaded from: " << filename << std::endl;
    return true;
}
