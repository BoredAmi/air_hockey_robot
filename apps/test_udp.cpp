#include "movement.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    try {
        MovementController controller;

        std::cout << "Connected to robot. Starting movement test..." << std::endl;


        std::cout << "Moving to center (50, 50)..." << std::endl;
        controller.moveTo(cv::Point2f(50.0f, 50.0f));
        std::this_thread::sleep_for(std::chrono::seconds(2));


        std::cout << "Moving to top-left (0, 0)..." << std::endl;
        controller.moveTo(cv::Point2f(0.0f, 0.0f));
        std::this_thread::sleep_for(std::chrono::seconds(2));


        std::cout << "Moving to top-right (100, 0)..." << std::endl;
        controller.moveTo(cv::Point2f(100.0f, 0.0f));
        std::this_thread::sleep_for(std::chrono::seconds(2));


        std::cout << "Moving to bottom-left (0, 100)..." << std::endl;
        controller.moveTo(cv::Point2f(0.0f, 100.0f));
        std::this_thread::sleep_for(std::chrono::seconds(2));


        std::cout << "Moving back to center (50, 50)..." << std::endl;
        controller.moveTo(cv::Point2f(50.0f, 50.0f));
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "Stopping robot..." << std::endl;
        controller.stop();

        std::cout << "Test completed successfully." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

