#ifndef MOVEMENT_HPP
#define MOVEMENT_HPP
//tcp communication with abb robot
#include "config.hpp"
#include <opencv2/opencv.hpp>
#include <string>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#endif

class MovementController {
public:
    MovementController();
    ~MovementController();
    void moveTo(cv::Point2f tablePosition);
    void stop();

private:
#ifdef _WIN32
    SOCKET robotSocket;
#else
    int robotSocket;
#endif
    bool connected;
    bool sendCommand(const std::string& command);
    bool connectToRobot();
    void disconnect();
};
#endif // MOVEMENT_HPP