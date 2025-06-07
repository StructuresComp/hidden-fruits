import cv2
import numpy as np

# Settings
pattern_size = (4, 11)  # (columns, rows)

image_path = "data/capture_20250424_154812/rgb_20250424_154815.png"
# Load image
image = cv2.imread(image_path)
if image is None:
    print(f"Failed to load image at {image_path}")
    exit(1)

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# === Try Asymmetric Circles Grid ===
found_circles, centers_circles = cv2.findCirclesGrid(
    gray, pattern_size, flags=cv2.CALIB_CB_ASYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING
)

if found_circles:
    cv2.drawChessboardCorners(image, pattern_size, centers_circles, found_circles)
    cv2.imwrite("circle_grid_detected.png", image)
    print("Asymmetric circle grid detected and saved as 'circle_grid_detected.png'")
else:
    # === Try Chessboard ===
    found_chess, corners_chess = cv2.findChessboardCorners(
        gray, (10,7), flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    if found_chess:
        cv2.drawChessboardCorners(image, (pattern_size), corners_chess, found_chess)
        cv2.imwrite("chessboard_detected.png", image)
        print("Chessboard pattern detected and saved as 'chessboard_detected.png'")
    else:
        print("No pattern detected (neither asymmetric circles nor chessboard).")
