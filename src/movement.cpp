#include "movement.hpp"
#include <iostream>
#include <sstream>
#include <stdexcept>

MovementController::MovementController() : robotSocket(
#ifdef _WIN32
    INVALID_SOCKET
#else
    -1
#endif

), connected(false) {
#ifdef _WIN32
    // Initialize Winsock
    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (result != 0) {
        throw std::runtime_error("WSAStartup failed: " + std::to_string(result));
    }
#endif

    // Connect to robot
    if (!connectToRobot()) {
#ifdef _WIN32
        WSACleanup();
#endif
        throw std::runtime_error("Failed to connect to robot");
    }

    // Send START command
    if (!sendCommand("START")) {
        disconnect();
#ifdef _WIN32
        WSACleanup();
#endif
        throw std::runtime_error("Failed to send START command");
    }
}

MovementController::~MovementController() {
    disconnect();
#ifdef _WIN32
    WSACleanup();
#endif
}

void MovementController::moveTo(cv::Point2f tablePosition) {
    if (!connected) return;

    // Convert table position to robot coordinates (mm)
    float x = tablePosition.x; 
    float y = tablePosition.y;

    std::stringstream ss;
    ss << "MOVE," << x << "," << y;
    sendCommand(ss.str());
}

void MovementController::stop() {
    if (!connected) return;
    sendCommand("STOP");
}

bool MovementController::connectToRobot() {
    struct addrinfo *result = NULL, *ptr = NULL, hints;

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;  
    hints.ai_protocol = IPPROTO_UDP;

    // Resolve the server address and port
    int iResult = getaddrinfo(ROBOT_IP.c_str(), "1025", &hints, &result);
    if (iResult != 0) {
        std::cerr << "getaddrinfo failed: " << iResult << std::endl;
        return false;
    }

    // Attempt to connect to an address until one succeeds
    for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {
        // Create a UDP socket
#ifdef _WIN32
        robotSocket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
        if (robotSocket == INVALID_SOCKET) {
            std::cerr << "socket failed: " << WSAGetLastError() << std::endl;
            freeaddrinfo(result);
            return false;
        }
#else
        robotSocket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
        if (robotSocket == -1) {
            std::cerr << "socket failed: " << strerror(errno) << std::endl;
            freeaddrinfo(result);
            return false;
        }
#endif

        // For UDP, store server address for later use
        memcpy(&serverAddr, ptr->ai_addr, ptr->ai_addrlen);
        serverAddrLen = ptr->ai_addrlen;
        break;
    }

    freeaddrinfo(result);

    if (
#ifdef _WIN32
        robotSocket == INVALID_SOCKET
#else
        robotSocket == -1
#endif
    ) {
        std::cerr << "Unable to create UDP socket!" << std::endl;
        return false;
    }

    connected = true;
    return true;
}

bool MovementController::sendCommand(const std::string& command) {
    if (!connected) return false;

    std::string cmd = command + "\r\n";
    
    // UDP send
    int iResult = sendto(robotSocket, cmd.c_str(), (int)cmd.length(), 0,
                        (struct sockaddr*)&serverAddr, serverAddrLen);
    if (iResult ==
#ifdef _WIN32
        SOCKET_ERROR
#else
        -1
#endif
    ) {
        std::cerr << "UDP send failed: " <<
#ifdef _WIN32
            WSAGetLastError()
#else
            strerror(errno)
#endif
            << std::endl;
        connected = false;
        return false;
    }

    return true;
}

void MovementController::disconnect() {
    if (
#ifdef _WIN32
        robotSocket != INVALID_SOCKET
#else
        robotSocket != -1
#endif
    ) {
#ifdef _WIN32
        closesocket(robotSocket);
        robotSocket = INVALID_SOCKET;
#else
        close(robotSocket);
        robotSocket = -1;
#endif
    }
    connected = false;
}