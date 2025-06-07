import sys
import pyzed.sl as sl
import cv2
import numpy as np
import os

def suppress_argus_errors():
    sys.stderr.flush()
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, 2)

suppress_argus_errors()

def main():
    # Output directory
    output_dir = os.path.join(os.path.dirname(__file__), 'data')
    os.makedirs(output_dir, exist_ok=True)

    # Create a Camera object
    zed = sl.Camera()

    # Set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL

    # Open the camera
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print("Camera Open Error:", status)
        sys.exit()

    runtime_parameters = sl.RuntimeParameters()
    image = sl.Mat()

    # Grab a single frame
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image, sl.VIEW.LEFT)
        img_np = image.get_data()
        img_rgb = cv2.cvtColor(img_np, cv2.COLOR_RGBA2RGB)

        # Save RGB image
        rgb_png_path = os.path.join(output_dir, "rgb_image.png")
        rgb_npy_path = os.path.join(output_dir, "rgb_image.npy")
        cv2.imwrite(rgb_png_path, img_rgb)
        np.save(rgb_npy_path, img_rgb)

        # Get depth map
        depth = sl.Mat()
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        depth_array = depth.get_data()

        # Save raw depth to .npy
        raw_depth_path = os.path.join(output_dir, "depth_raw.npy")
        np.save(raw_depth_path, depth_array)

        # Clean + visualize depth
        depth_clean = np.nan_to_num(depth_array, nan=0.0, posinf=0.0, neginf=0.0)
        depth_clipped = np.clip(depth_clean, 500, 5000)
        depth_vis = cv2.normalize(depth_clipped, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

        # Save colorized depth
        depth_vis_path = os.path.join(output_dir, "depth_vis.png")
        cv2.imwrite(depth_vis_path, depth_colored)

        print("Saved:")
        print("  RGB image:     ", rgb_png_path)
        print("  RGB .npy:      ", rgb_npy_path)
        print("  Depth raw:     ", raw_depth_path)
        print("  Depth visual:  ", depth_vis_path)

    zed.close()

if __name__ == "__main__":
    main()
