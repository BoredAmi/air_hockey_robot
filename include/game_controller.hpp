#ifndef GAME_CONTROLLER_HPP
#define GAME_CONTROLLER_HPP

#include "capture.hpp"
#include "trajectory.hpp"
#include "movement.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <ctime>

struct DebugRenderParams {
    cv::Mat frame;
    cv::Point2f puckCenter;
    cv::Point2f predictedShort;
    cv::Point2f predictedEntryTable;
    cv::Point2f robotPos;
    double velocityConfidence;
    double fps;
    uint64_t currentTimeUs;
    int& debugImageIndex;
    ImageCapture& capture;
    TrajectoryPredictor& predictor;
};

class GameController {
public:
    GameController(const Config& config);
    void renderDebugImage(const DebugRenderParams& params);

private:
    Config config_;
};

#endif // GAME_CONTROLLER_HPP