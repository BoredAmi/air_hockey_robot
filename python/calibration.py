import cv2
import cv2.aruco as aruco
import numpy as np
import yaml
from picamera2 import Picamera2

# --- BOARD CONFIGURATION (According to obraz.png) ---
ROWS = 8            # Number of squares vertically
COLS = 11           # Number of squares horizontally
SQUARE_LEN = 0.015  # 15mm 
MARKER_LEN = 0.011  # 11mm
# Using DICT_4X4_50 dictionary (most popular for ChArUco)
DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# Board and detector definition
BOARD = aruco.CharucoBoard((COLS, ROWS), SQUARE_LEN, MARKER_LEN, DICT)
DETECTOR = aruco.CharucoDetector(BOARD)

# Display settings (window scaling on screen)
DISPLAY_SCALE = 0.4  # 0.4 at 1640px width gives ~650px window

# --- CAMERA INITIALIZATION ---
picam2 = Picamera2()
# Configuration according to your parameters
config = picam2.create_video_configuration(main={
    "size": (1640, 1232), 
    "format": "BGR888"
})
picam2.configure(config)
picam2.start()

all_charuco_corners = []
all_charuco_ids = []

print(f"Started ChArUco calibration ({COLS}x{ROWS})")
print("Resolution: 1640x1232, Format: BGR")
print("SPACE: save frame | Q: calculate and save YAML")

try:
    while True:
        # Capture frame (already in BGR format)
        frame = picam2.capture_array()
        
        # Detect board
        charuco_corners, charuco_ids, marker_corners, marker_ids = DETECTOR.detectBoard(frame)

        # Prepare preview
        preview = frame.copy()
        
        if charuco_ids is not None and len(charuco_ids) > 0:
            # Draw detected points
            aruco.drawDetectedCornersCharuco(preview, charuco_corners, charuco_ids)
            # Green frame signaling detection
            cv2.rectangle(preview, (10, 10), (1630, 1222), (0, 255, 0), 15)

        # Information about number of collected frames
        cv2.putText(preview, f"Frames: {len(all_charuco_corners)}", (50, 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 4)

        # Scale preview window
        view_w = int(preview.shape[1] * DISPLAY_SCALE)
        view_h = int(preview.shape[0] * DISPLAY_SCALE)
        scaled_view = cv2.resize(preview, (view_w, view_h))
        
        cv2.imshow("Calibration Preview", scaled_view)

        key = cv2.waitKey(1) & 0xFF
        # Save only if at least 6 corners detected (minimum for stability)
        if key == ord(' ') and charuco_ids is not None and len(charuco_ids) > 5:
            all_charuco_corners.append(charuco_corners)
            all_charuco_ids.append(charuco_ids)
            print(f"Added frame {len(all_charuco_corners)}")
            
            # Visual save effect
            cv2.imshow("Calibration Preview", cv2.bitwise_not(scaled_view))
            cv2.waitKey(100)
            
        elif key == ord('q'):
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()

# --- CALCULATIONS AND SAVING ---
if len(all_charuco_corners) >= 12:
    print("\nCalculating optical parameters... Please wait.")
    # ChArUco calibration
    ret, mtx, dist, rvecs, tvecs = aruco.calibrateCameraCharuco(
        charucoCorners=all_charuco_corners,
        charucoIds=all_charuco_ids,
        board=BOARD,
        imageSize=(1640, 1232),
        cameraMatrix=None,
        distCoeffs=None
    )

    # Prepare data for YAML file
    calib_results = {
        "rms_error": float(ret),
        "camera_matrix": mtx.tolist(),
        "distortion_coefficients": dist.tolist(),
        "resolution": [1640, 1232],
        "pattern": f"Charuco {COLS}x{ROWS}"
    }
    # Save it in build folder for easy access by C++ code
    with open("../build/calibration_result.yaml", "w") as f:
        yaml.dump(calib_results, f, default_flow_style=False)

    print("-" * 30)
    print(f"CALIBRATION COMPLETED!")
    print(f"RMS Error: {ret:.4f}")
    print("Result saved in: calibration_result.yaml")
    print("-" * 30)
else:
    print("\n[ERROR] Not enough data. Collect at least 12-15 varied frames.")