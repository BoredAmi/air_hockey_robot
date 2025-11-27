#ifndef CONFIG_HPP
#define CONFIG_HPP

// Camera configuration
const int CAMERA_INDEX = 0;  // Camera device index

// Calibration parameters
const int CHESSBOARD_WIDTH = 9;   // Number of internal corners per row
const int CHESSBOARD_HEIGHT = 6;  // Number of internal corners per column
const float SQUARE_SIZE = 25.0f;  // Size of chessboard squares in mm

// Image dimensions (pixels)
const int TABLE_WIDTH = 640;
const int TABLE_HEIGHT = 480;

// Physical table dimensions (mm)
const float PHYSICAL_TABLE_WIDTH = 1000.0f;  // 1 meter
const float PHYSICAL_TABLE_HEIGHT = 500.0f;  // 0.5 meters

// Puck parameters
const int PUCK_RADIUS_REAL = 15;  // Average puck radius in mm
const int PUCK_RADIUS_MIN = 10;
const int PUCK_RADIUS_MAX = 30;
const int PUCK_THRESHOLD = 100;  // Threshold for black puck detection
const int PUCK_MIN_AREA = 50; // Minimum area for blob detection
const int PUCK_MAX_AREA = 10000; // Maximum area for blob detection

#endif // CONFIG_HPP