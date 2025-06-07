import sys
import os
import time
import pyzed.sl as sl
import cv2
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from datetime import datetime

# Suppress Argus socket errors
def suppress_argus_errors():
    sys.stderr.flush()
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, 2)

suppress_argus_errors()

def create_output_dir():
    output_dir = os.path.join(os.path.dirname(__file__), "data", "position_tracking")
    os.makedirs(output_dir, exist_ok=True)
    return output_dir

def save_plot(positions, output_dir):
    positions = np.array(positions)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], marker='o')
    ax.set_title('ZED Camera 3D Trajectory')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    plt.savefig(os.path.join(output_dir, 'trajectory_plot.png'))
    plt.close()

def main():
    output_dir = create_output_dir()
    position_log = []

    # Initialize ZED camera
    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.sdk_verbose = 0  # Suppress verbose output

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("ZED failed to open.")
        exit()

    tracking_params = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(tracking_params)

    runtime_params = sl.RuntimeParameters()

    image = sl.Mat()
    pose = sl.Pose()

    num_captures = 100  # Change this to capture more/less frames
    for i in range(num_captures):
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.get_position(pose, sl.REFERENCE_FRAME.WORLD)

            translation = pose.get_translation().get()
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"rgb_{timestamp}.png"
            filepath = os.path.join(output_dir, filename)

            # Save image
            img_np = image.get_data()
            img_rgb = cv2.cvtColor(img_np, cv2.COLOR_RGBA2RGB)
            cv2.imwrite(filepath, img_rgb)

            # Log position
            print(f"Captured: {filename} at position {translation}")
            position_log.append([filename, *translation])

            time.sleep(1)

    # Save coordinates to CSV
    df = pd.DataFrame(position_log, columns=["filename", "x", "y", "z"])
    df.to_csv(os.path.join(output_dir, "positions.csv"), index=False)

    # Create 3D plot
    save_plot(df[["x", "y", "z"]].values, output_dir)

    zed.disable_positional_tracking()
    zed.close()
    print("Done. All data saved to:", output_dir)

if __name__ == "__main__":
    main()
