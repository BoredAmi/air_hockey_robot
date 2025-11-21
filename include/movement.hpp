#ifndef MOVEMENT_HPP
#define MOVEMENT_HPP
#include <opencv2/opencv.hpp>
class MovementController {
public:
    MovementController();
    void moveTo(cv::Point2f tablePosition);
    void stop();
};
#endif // MOVEMENT_HPP