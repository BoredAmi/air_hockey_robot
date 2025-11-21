#include "movement.hpp"
MovementController::MovementController() {
    // Initialize movement controller (e.g., setup communication with motors)
}
void MovementController::moveTo(cv::Point2f tablePosition) {
    // Convert tablePosition (in mm) to motor commands
    // Send commands to motors to move to the specified position
    
}
void MovementController::stop() {
    // Send command to stop all motor movement
}