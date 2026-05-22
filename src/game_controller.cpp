#include "game_controller.hpp"

GameController::GameController(const Config& config) : config_(config) {}

void GameController::renderDebugImage(const DebugRenderParams& params) {
    try {
        cv::Mat debugImg = params.frame.clone();

        // Draw puck center
        cv::circle(debugImg, params.puckCenter, 6, cv::Scalar(0, 0, 255), -1);

        // Short-horizon predicted position and velocity
        double vx = 0.0, vy = 0.0, speed = 0.0;
        if (params.predictedShort.x >= 0 && params.predictedShort.y >= 0) {
            cv::Point2f predictedShortImg = params.capture.TableToImageCoordinates(params.predictedShort, params.capture.getCroppedWidth(), params.capture.getCroppedHeight());
            cv::arrowedLine(debugImg, params.puckCenter, predictedShortImg, cv::Scalar(0, 255, 255), 2, cv::LINE_AA, 0, 0.2);
            cv::Point2f puckTablePos = params.capture.imageToTableCoordinates(params.puckCenter, params.capture.getCroppedWidth(), params.capture.getCroppedHeight());
            vx = (params.predictedShort.x - puckTablePos.x) * 10.0;
            vy = (params.predictedShort.y - puckTablePos.y) * 10.0;
            speed = std::hypot(vx, vy);
        }

        // Draw predicted path 
        uint64_t predictedEntryTimeUs = 0;
        const uint64_t maxLookaheadUs = 2000000; // 2 seconds
        const uint64_t stepUs = 50000; // 50 ms
        std::vector<cv::Point> pathPoints;
        for (uint64_t t = params.currentTimeUs; t <= params.currentTimeUs + maxLookaheadUs; t += stepUs) {
            cv::Point2f p = params.predictor.predictPosition(t);
            if (p.x < 0 || p.y < 0) continue;
            cv::Point imgPt = params.capture.TableToImageCoordinates(p, params.capture.getCroppedWidth(), params.capture.getCroppedHeight());
            pathPoints.push_back(imgPt);
            if (params.predictor.isInDefenseZone(p) && predictedEntryTimeUs == 0) {
                predictedEntryTimeUs = t;
            }
        }
        for (size_t i = 1; i < pathPoints.size(); ++i) {
            cv::line(debugImg, pathPoints[i-1], pathPoints[i], cv::Scalar(0, 255, 0), 1);
        }

        // Draw predicted entry point
        cv::Point2f predictedImage = params.capture.TableToImageCoordinates(params.predictedEntryTable, params.capture.getCroppedWidth(), params.capture.getCroppedHeight());
        if (predictedImage.x >= 0 && predictedImage.y >= 0) {
            cv::circle(debugImg, predictedImage, 8, cv::Scalar(255, 0, 0), 2);
        }

        // Put multiple text lines: index, fps, confidence, speed, time-to-entry, predicted/robot coords
        std::ostringstream line1, line2, line3, line4;
        line1 << "Idx:" << std::setw(4) << std::setfill('0') << params.debugImageIndex << "  FPS:" << std::fixed << std::setprecision(1) << params.fps;
        line2 << "Conf:" << std::fixed << std::setprecision(2) << params.velocityConfidence << "  V(mm/s):" << std::fixed << std::setprecision(1) << speed;
        if (predictedEntryTimeUs > 0) {
            double timeUntilMs = (predictedEntryTimeUs - params.currentTimeUs) / 1000.0;
            line2 << "  Tentry(ms):" << std::fixed << std::setprecision(0) << timeUntilMs;
        } else {
            line2 << "  Tentry(ms):unknown";
        }

        line3 << "Pred(mm):" << std::fixed << std::setprecision(0) << params.predictedEntryTable.x << "," << params.predictedEntryTable.y;
        line3 << "  Robot(mm):" << std::fixed << std::setprecision(0) << params.robotPos.x << "," << params.robotPos.y;

        line4 << "ShortPred(mm):" << std::fixed << std::setprecision(0) << params.predictedShort.x << "," << params.predictedShort.y;

        auto nowSys = std::chrono::system_clock::now();
        std::time_t nowTime = std::chrono::system_clock::to_time_t(nowSys);
        std::tm localTime = {};
#if defined(_WIN32) || defined(_WIN64)
        localtime_s(&localTime, &nowTime);
#else
        localtime_r(&nowTime, &localTime);
#endif
        auto msPart = std::chrono::duration_cast<std::chrono::milliseconds>(nowSys.time_since_epoch()).count() % 1000;
        std::ostringstream timeStamp;
        timeStamp << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S") << "." << std::setw(3) << std::setfill('0') << msPart;

        int imgX = 10;
        int y = 30;
        int lineH = 28;
        cv::putText(debugImg, line1.str(), cv::Point(imgX, y), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        cv::putText(debugImg, line2.str(), cv::Point(imgX, y + lineH), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        cv::putText(debugImg, line3.str(), cv::Point(imgX, y + lineH*2), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        cv::putText(debugImg, line4.str(), cv::Point(imgX, y + lineH*3), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        cv::putText(debugImg, timeStamp.str(), cv::Point(imgX, y + lineH*4), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);

        std::string outDir = "predicted_entries";
        std::error_code ec;
        std::filesystem::create_directories(outDir, ec);
        if (ec) {
            std::cerr << "Warning: could not create directory '" << outDir << "': " << ec.message() << std::endl;
        }
        std::ostringstream fname;
        fname << outDir << "/predicted_entry_" << std::setw(4) << std::setfill('0') << params.debugImageIndex << ".png";
        params.capture.saveImage(debugImg, fname.str());
        
        params.debugImageIndex++;
    } catch (const std::exception& e) {
        std::cerr << "Failed to save predicted-entry image: " << e.what() << std::endl;
    }
}
