import pyzed.sl as sl

import os
import sys
import time
import datetime
import numpy as np
import cv2
import matplotlib.pyplot as plt
import pandas as pd
import subprocess
import serial
import struct

from vmbpy import VmbSystem, PixelFormat
# TODO: Edit Num Frames & Base Dir
num_frames = 300
# Suppress Argus socket errors
def suppress_argus_errors():
    sys.stderr.flush()
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, 2)

suppress_argus_errors()

### Set up timestamped output dir
def create_output_dir():
    # base_dir = os.path.join(os.path.dirname(__file__), "data")
    base_dir = "/media/scifruit/ANDY"

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.join(base_dir, f"capture_{timestamp}")
    os.makedirs(output_dir, exist_ok=True)
    return output_dir

### Save ZED camera position plot
def save_3d_plot(positions, out_dir):
    pos = np.array(positions)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], marker='o')
    ax.set_title('ZED 3D Trajectory')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    plt.savefig(os.path.join(out_dir, "trajectory_plot.png"))
    plt.close()

### Capture from ZED camera
def capture_zed(zed, output_dir, runtime_parameters, pose_log):
    img = sl.Mat()
    depth = sl.Mat()
    pose = sl.Pose()

    if zed.grab(runtime_parameters) != sl.ERROR_CODE.SUCCESS:
        return

    zed.retrieve_image(img, sl.VIEW.LEFT)
    img_np = img.get_data()
    img_rgb = cv2.cvtColor(img_np, cv2.COLOR_RGBA2RGB)

    zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
    depth_array = depth.get_data()

    zed.get_position(pose, sl.REFERENCE_FRAME.WORLD)
    position = pose.get_translation().get()

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    cv2.imwrite(os.path.join(output_dir, f"rgb_{timestamp}.png"), img_rgb)
    np.save(os.path.join(output_dir, f"rgb_{timestamp}.npy"), img_rgb)
    np.save(os.path.join(output_dir, f"depth_raw_{timestamp}.npy"), depth_array)

    depth_clean = np.nan_to_num(depth_array, nan=0.0, posinf=0.0, neginf=0.0)
    depth_clipped = np.clip(depth_clean, 0.5, 5)
    depth_vis = cv2.normalize(depth_clipped, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
    cv2.imwrite(os.path.join(output_dir, f"depth_vis_{timestamp}.png"), depth_color)

    pose_log.append([f"rgb_{timestamp}.png", *position])

### Capture from NIR camera
def capture_nir(output_dir):
    with VmbSystem.get_instance() as vmb:
        cameras = vmb.get_all_cameras()
        if not cameras:
            print("No NIR camera found.")
            return
        with cameras[0] as cam:
            cam.set_pixel_format(PixelFormat.Mono8)
            cam.ExposureAuto.set("Continuous")
            # cam.ExposureTime.set(150000.0)
            # cam.get_feature_by_name("GainAuto").set("Off")
            # cam.get_feature_by_name("Gain").set(17.0)
            cam.get_feature_by_name("GainAuto").set("Continuous")
            frame = cam.get_frame()
            image = frame.as_opencv_image()
            image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)

            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(os.path.join(output_dir, f"nir_{timestamp}.png"), image)
            np.save(os.path.join(output_dir, f"nir_{timestamp}.npy"), image)

### Capture from FLIR Lepton over serial
def capture_lwir(output_dir):
    device_found = False
    for i in range(5):
        port = f'/dev/ttyUSB{i}'
        try:
            subprocess.run(['sudo', 'chmod', 'a+rw', port], check=True)
            ser = serial.Serial(port, 921600)
            device_found = True
            break
        except Exception:
            continue
    if not device_found:
        print("No LWIR device found.")
        return

    width, height = 160, 120
    marker = b'\xFF\xD8'
    buffer = bytearray()
    while True:
        byte = ser.read(1)
        if not byte:
            return
        buffer += byte
        if len(buffer) > len(marker):
            buffer.pop(0)
        if bytes(buffer) == marker:
            break
    data = ser.read(width * height)
    pixels = np.frombuffer(data, dtype=np.uint8).reshape((height, width))
    pixels = np.rot90(pixels, 2)

    flat = [val for row in pixels for val in row]
    bmp_name = os.path.join(output_dir, f"lwir_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.bmp")
    save_bw_bmp(bmp_name, width, height, flat)

def save_bw_bmp(filename, width, height, grayscale_values):
    import struct
    row_padding = (4 - (width % 4)) % 4
    row_size = width + row_padding
    pixel_array_size = row_size * height

    file_header = struct.pack('<2sIHHI', b'BM', 14 + 40 + 1024 + pixel_array_size, 0, 0, 14 + 40 + 1024)
    info_header = struct.pack('<IIIHHIIIIII', 40, width, height, 1, 8, 0, pixel_array_size, 2835, 2835, 256, 0)
    pixels = bytearray()
    for y in range(height - 1, -1, -1):
        row = grayscale_values[y * width : (y + 1) * width]
        pixels += bytes(row)
        pixels += b'\x00' * row_padding
    with open(filename, 'wb') as f:
        f.write(file_header)
        f.write(info_header)
        for i in range(256):
            f.write(struct.pack('BBBB', i, i, i, 0))
        f.write(pixels)

### Main execution
def main():
    output_dir = create_output_dir()
    pose_log = []

    # Set up ZED
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.sdk_verbose = 0
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("ZED open failed.")
        return
    zed.enable_positional_tracking(sl.PositionalTrackingParameters())
    runtime_params = sl.RuntimeParameters()

    for i in range(num_frames):
        print(f"\n=== Capture {i + 1} ===")
        capture_zed(zed, output_dir, runtime_params, pose_log)
        capture_nir(output_dir)
        capture_lwir(output_dir)

    zed.disable_positional_tracking()
    zed.close()

    df = pd.DataFrame(pose_log, columns=["filename", "x", "y", "z"])
    df.to_csv(os.path.join(output_dir, "positions.csv"), index=False)
    save_3d_plot(df[["x", "y", "z"]].values, output_dir)

    print("\nAll data saved in:", output_dir)

if __name__ == "__main__":
    main()
