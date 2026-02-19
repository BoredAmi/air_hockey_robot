#include "capture.hpp"
#include <iostream>

ImageCapture::ImageCapture(int cameraIndex) : cameraIndex_(cameraIndex), croppedWidth_(TABLE_WIDTH), croppedHeight_(TABLE_HEIGHT), tableFound_(false), matrixCached_(false) {}

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
        cv::RotatedRect tableRotated;
        cv::Rect tableRect;
        cv::Mat perspectiveMatrix;
        cv::Size outputSize;
        if (!matrixCached_ || !tableFound_) {
            tableRotated = detectTable(frame);
            tableRect = tableRotated.boundingRect();
            if (tableRect.area() > 0) {
                // Cache the values
                cachedTableRect_ = tableRect;
                cachedOutputSize_ = cv::Size(tableRotated.size.width, tableRotated.size.height);
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
                cachedPerspective_ = cv::getPerspectiveTransform(srcPoints, dstPoints);
                if (tableFound_){
                    matrixCached_ = true;
                }
            }
        } else {
            tableRect = cachedTableRect_;
            perspectiveMatrix = cachedPerspective_;
            outputSize = cachedOutputSize_;
        }
        if (tableRect.area() > 0) {
            // Crop to bounding rect
            cv::Mat cropped = frame(tableRect);
            // Warp to straighten the table
            cv::warpPerspective(cropped, frame, cachedPerspective_, cachedOutputSize_);
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
    cv::threshold(grayImage, thresh, PUCK_THRESHOLD, 255, cv::THRESH_BINARY_INV);  // Invert for black puck

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < PUCK_MIN_AREA || area > PUCK_MAX_AREA) continue;

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
    if (imageWidth == 0) imageWidth = TABLE_WIDTH;
    if (imageHeight == 0) imageHeight = TABLE_HEIGHT;
    // First, undistort the point if calibration is available
    cv::Point2f undistortedPoint = undistortPoint(imagePoint);

    // Scale factors from image pixels to physical units (mm)
    float scaleX = PHYSICAL_TABLE_WIDTH / imageWidth;
    float scaleY = PHYSICAL_TABLE_HEIGHT / imageHeight;

    // Origin at bottom-left corner of table
    float tableX = undistortedPoint.x * scaleX;
    float tableY = (imageHeight - undistortedPoint.y) * scaleY;  // Y increases up in table

    
    return cv::Point2f(tableX, tableY);
}
cv::Point2f ImageCapture::TableToImageCoordinates(cv::Point2f tablePoint, int imageWidth, int imageHeight) {
    if (imageWidth == 0) imageWidth = TABLE_WIDTH;
    if (imageHeight == 0) imageHeight = TABLE_HEIGHT;
    // Scale factors from physical units (mm) to image pixels
    float scaleX = imageWidth / PHYSICAL_TABLE_WIDTH;
    float scaleY = imageHeight / PHYSICAL_TABLE_HEIGHT;

    // Convert table coordinates back to image coordinates (origin at bottom-left)
    float imageX = tablePoint.x * scaleX;
    float imageY = imageHeight - tablePoint.y * scaleY;  // Y flip back

    cv::Point2f imagePoint(imageX, imageY);

    // Undistort the point if calibration is available
    if (!cameraMatrix_.empty() && !distCoeffs_.empty()) {
        std::vector<cv::Point2f> points = {imagePoint};
        cv::undistortPoints(points, points, cameraMatrix_, distCoeffs_, cv::noArray(), cameraMatrix_);
        imagePoint = points[0];
    }

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
