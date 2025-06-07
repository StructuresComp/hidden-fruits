import datetime
import serial

import subprocess
device_found = False
for i in range(5):
    device_loc = f'/dev/ttyUSB{i}'
    try:
        subprocess.run(['sudo', 'chmod', 'a+rw', device_loc], check=True)
        ser = serial.Serial(device_loc, 921600)
        print(f"Connected to {device_loc}")
        device_found = True
        break  # stop once we've connected
    except (serial.SerialException, FileNotFoundError) as e:
        print(f"Failed to open {device_loc}: {e}")
    except subprocess.CalledProcessError as e:
        print(f"Permission change failed for {device_loc}: {e}")

if not device_found:
    print("Could not find a working /dev/ttyUSB[0-4] port.")



# Define the image size (160x120)
width = 160
height = 120
total_pixels = width * height
def sync_to_marker(ser, marker=b'\xFF\xD8'):
    """Sync to the binary marker (e.g., b'\xFF\xD8')"""
    buffer = bytearray()
    while True:
        byte = ser.read(1)
        if not byte:
            raise IOError("Serial connection lost or timed out.")
        buffer += byte
        if len(buffer) > len(marker):
            buffer.pop(0)  # keep buffer same length as marker
        if bytes(buffer) == marker:
            return

import matplotlib.pyplot as plt
import numpy as np

prev_time = datetime.datetime.now()

# Persistent figure setup
fig, ax = plt.subplots()
img = ax.imshow(np.zeros((120, 160)), cmap='gray', vmin=0, vmax=255)
plt.ion()  # Enable interactive mode
plt.show()


# Function to read and save showa BMP image
def receive_image():
    data = ser.read(width * height)  # read 19200 bytes
    pixels = np.frombuffer(data, dtype=np.uint8).reshape((height, width))
    
    #rotate pixels 180 degrees with np
    pixels = np.rot90(pixels, 2)

    flat_pixels = [val for row in pixels for val in row]
    # show_image(pixels)
    write_bw_bmp("lwir_image.bmp", 160, 120, flat_pixels)


        
def write_bw_bmp(filename, width, height, grayscale_values):
    global prev_time

    """
    Save an 8-bit grayscale BMP image of given width x height.
    grayscale_values: flat list of ints in [0–255], row-major order, bottom-up.
    """
    import struct

    # Pad rows to multiple of 4 bytes
    row_padding = (4 - (width % 4)) % 4
    row_size = width + row_padding
    pixel_array_size = row_size * height

    # File header (14 bytes)
    file_header = struct.pack('<2sIHHI',
        b'BM',
        14 + 40 + 1024 + pixel_array_size,  # Total file size
        0, 0,
        14 + 40 + 1024  # Offset to pixel array
    )

    # DIB header (40 bytes)
    info_header = struct.pack('<IIIHHIIIIII',
        40,          # DIB header size
        width,
        height,
        1,           # planes
        8,           # bits per pixel (grayscale)
        0,           # no compression
        pixel_array_size,
        2835, 2835,  # resolution
        256,         # colors in palette
        0            # all colors are important
    )

    # Grayscale color palette
    palette = bytearray()
    for i in range(256):
        palette += struct.pack('BBBB', i, i, i, 0)  # B, G, R, 0

    # Pixel array (bottom-up)
    pixels = bytearray()
    for y in range(height - 1, -1, -1):  # BMP is bottom-up
        row = grayscale_values[y * width : (y + 1) * width]
        pixels += bytes(row)
        pixels += b'\x00' * row_padding

    # Write to file
    # calculate fps
    
    with open(filename, 'wb') as f:
        now = datetime.datetime.now()
        time_diff = (now - prev_time).total_seconds()
        if time_diff > 0:
            print("FPS:", 1 / time_diff)
        prev_time = now
        f.write(file_header)
        f.write(info_header)
        f.write(palette)
        f.write(pixels)


print("Program started, waiting for OK...")
while True:
    try:
        
        sync_to_marker(ser)
        print("Image received")
        receive_image()
    except serial.SerialException as e:
        print(f"Serial error: {e}")

    except UnicodeDecodeError as e:
        pass
        #print(f"Unicode decode error: {e}")


