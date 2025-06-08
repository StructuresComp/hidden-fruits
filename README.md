# Hidden Fruits

*A multi‑modal data‑capture rig combining visible, NIR and long‑wave infrared imaging with 6‑DoF pose tracking.*

Hidden Fruits is an experimental platform from **Structures Computers Lab** that pairs a **NVIDIA Jetson Orin** host with an **ESP32‑S3** sensor node to synchronously record:

* 1280×720 RGB + depth from a **ZED 2** stereo camera
* 160×120 long‑wave infrared (LWIR) frames from a **FLIR Lepton 3.x**
* Monochrome NIR stills from an industrial USB3 vision camera
* 6‑DoF world‑space poses from the ZED tracking pipeline

The goal is to produce temporally‑aligned datasets for machine perception research — especially projects that fuse thermal, NIR and RGB cues.

---

## Hardware

| Component                                           | Purpose                                                    |
| --------------------------------------------------- | ---------------------------------------------------------- |
| **NVIDIA Jetson Orin Nano/AGX**                     | Edge host that runs the Python capture script and ZED SDK  |
| **StereoLabs ZED 2 (or 2i)**                        | Provides RGB, depth and camera pose                        |
| **ESP32‑S3 + FLIR Lepton 3.x breakout**             | Streams raw 14‑bit VO‑SPI frames over UART at 921 600 baud |
| **USB3 NIR camera** (e.g. Allied Vision Mako G‑031) | Captures monochrome images in the 850–940 nm range         |
| (Optional) External IMU                             | Extra inertial data channel                                |

---

## Repository layout

```
.
├── esp32_firmware/        # Arduino sketch for the Lepton serial bridge
├── jetson_capture/        # Python3 capture pipeline (ZED + NIR + LWIR)
├── hardware/              # Schematics & wiring diagrams
└── docs/                  # Additional design notes / datasheets
```

---

## Quick start

### 1 · Clone & set up

```bash
git clone --recurse-submodules https://github.com/Structures-Computers-Lab/hidden-fruits.git
cd hidden-fruits
```

Create a Python 3.10 environment and install host dependencies
### 2 · Flash the ESP32

The firmware lives in `esp32_firmware/` and can be built with **PlatformIO** or the Arduino IDE:

```bash
pio run -e esp32-s3-dev
pio run -e esp32-s3-dev -t upload
```

The sketch configures HSPI, I²C and the VO‑SPI GPIOs, then begins streaming normalised 8‑bit frames (`0xFF 0xD8` header + 19 200 bytes) at \~8 Hz.

### 3 · Run a capture session

Per‑frame artefacts are written to a timestamped folder:

```
└── capture_20250607_173212
    ├── rgb_*.png / .npy
    ├── depth_raw_*.npy   # float32 depth map (m)
    ├── depth_vis_*.png   # false‑colour preview
    ├── nir_*.png / .npy
    ├── lwir_*.bmp        # 8‑bit greyscale 160×120
    ├── positions.csv     # filename,x,y,z (metres, RHS‑Y‑Up)
    └── trajectory_plot.png
```

---

## Data format

| Channel     | Resolution       | Encoding            | File type |
| ----------- | ---------------- | ------------------- | --------- |
| RGB‑Left    | 1280 × 720       | 8‑bit RGB           | PNG & NPY |
| Depth (raw) | 1280 × 720       | 32‑bit float metres | NPY       |
| Depth (vis) | 1280 × 720       | 8‑bit colour‑map    | PNG       |
| NIR         | camera‑dependent | 8‑bit Mono          | PNG & NPY |
| LWIR        | 160 × 120        | 8‑bit Mono          | BMP       |
| Pose        | *n* × 3          | metres              | CSV       |

All files share the same millisecond‑level timestamp embedded in their filename, enabling perfect alignment in post‑processing.

---

## Calibration

1. **Stereo intrinsics** come pre‑calibrated from StereoLabs.
2. **Lepton spatial alignment** can be derived from a planar chequerboard heated by a soldering iron.
3. **NIR ↔ RGB extrinsics** — capture a board printed with RetroReflective ink and solve with OpenCV’s `findChessboardCorners`.

Example calibration notebooks live in `docs/calibration/`.

---

## Troubleshooting

| Symptom                             | Remedy                                                                 |
| ----------------------------------- | ---------------------------------------------------------------------- |
| ZED grab timeout                    | Confirm USB‑C 3.1 cable & SDK version                                  |
| `No LWIR device found.`             | Check `sudo udevadm` rules; the USB‑UART must appear as `/dev/ttyUSB*` |
| Checkerboard detection fails in NIR | Increase exposure / illuminate with 850 nm LED panel                   |

---

## Contributing

Pull requests are welcome! Please run `pre-commit` hooks and conform to the existing clang‑format / `ruff` style before submitting.

---

## License

© 2025 Structures Computers Lab. This project is released under the **MIT License**; see [`LICENSE`](LICENSE) for details.

---

## Acknowledgements

* [`ducky64/arduino-lepton`](https://github.com/ducky64/arduino-lepton) — base VO‑SPI driver on which the ESP32 sketch is built.
* StereoLabs for their open‑source ZED SDK samples.
* Allied Vision Vimba SDK.

---

*We’d love to see what you build with Hidden Fruits — tag @structurescomputers on social media!*
