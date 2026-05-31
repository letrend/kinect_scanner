# Kinect 3D Scanner

CUDA TSDF/KinectFusion-style 3D scanner for Kinect v2, with a Qt GUI, optional
actuated turntable scanning, TCP control, and a built-in simulation mode.

The scanner can run in three pose modes:

- `ICP`: live Kinect frames with ICP-based camera tracking.
- `Actuated TCP`: fixed Kinect pose generated from commanded turntable angle and
  linear-stage position.
- `Simulation`: synthetic Kinect depth/RGB from an STL object, or a default cube
  when no STL is provided.

## Dependencies

- CMake 3.10+
- C++11 compiler
- CUDA toolkit and an NVIDIA GPU
- OpenCV core/highgui/imgproc/calib3d
- Eigen3
- libfreenect2
- libusb
- ncurses
- Boost system
- Qt5 Core/Gui/Widgets/OpenGL/Network for the GUI
- Sophus headers, either system-installed or in `third_party/Sophus`

On Ubuntu-like systems the package names are typically:

```bash
sudo apt install cmake build-essential pkg-config libeigen3-dev \
  libopencv-dev libusb-1.0-0-dev libncurses5-dev libboost-system-dev \
  qtbase5-dev libqt5opengl5-dev
```

Install `libfreenect2` separately from the OpenKinect project if your distro
does not provide a usable package.

## Build

```bash
mkdir -p build data
cd build
cmake ..
cmake --build . -j
```

The GUI is built by default. To build only the legacy CLI:

```bash
cmake .. -DBUILD_GUI=OFF
cmake --build . -j
```

CUDA architecture is auto-detected with `nvidia-smi`. You can override it:

```bash
cmake .. -DCUDA_ARCH=75
```

## Run

Launch the GUI:

```bash
./build/3D-kinect-scanner
```

Run with simulated RGB/depth and a default cube:

```bash
./build/3D-kinect-scanner --simulate
```

Run simulation with a specific STL:

```bash
./build/3D-kinect-scanner --simulate --sim-stl /path/to/object.stl
```

Run with actuator TCP pose source:

```bash
./build/3D-kinect-scanner --actuated --actuator-tcp 0.0.0.0:5055
```

Run the legacy ncurses/OpenCV CLI:

```bash
./build/3D-kinect-scanner --cli
```

Useful GUI options:

```text
--simulate                 Use simulated Kinect frames
--actuated                 Use actuator TCP pose source
--sim-stl PATH             STL object for simulation
--control-tcp [HOST:]PORT  UI control endpoint, default 127.0.0.1:5056
--actuator-tcp [HOST:]PORT Actuator endpoint, default 0.0.0.0:5055
```

## GUI Workflow

1. Choose `Mode` in the Parameters dock: `ICP`, `Actuated TCP`, or `Simulation`.
2. Configure volume size, voxel size, scan path, turntable geometry, and Kinect
   relative pose.
3. Press `Reset` after changing fields marked as reset-required.
4. Press `Start` to scan.
5. Press `Extract mesh`, then `Save mesh` to export a PLY.

In simulation mode, the viewer shows the source object and turntable. If `STL
path` is empty, the simulator uses an in-memory default cube.

For actuated scans, the scanner sends target poses to the actuator TCP client
and waits for feedback before integrating frames.

## TCP Control

All TCP messages are newline-delimited JSON.

- UI control server: `127.0.0.1:5056` by default
- Actuator server: `0.0.0.0:5055` by default

Examples:

```json
{"type":"start","seq":1}
{"type":"set_params","params":{"poseSource":"simulation","angleStepDeg":5},"seq":2}
{"type":"save_mesh","path":"/tmp/scan.ply","seq":3}
```

See [docs/tcp_protocol.md](docs/tcp_protocol.md) for the full message formats.

## Simulation Notes

Simulation generates artificial Kinect-sized depth/RGB images from the selected
mesh and integrates them through the same TSDF pipeline as live data. The
turntable scan path rotates around the scan volume center.

For clean simulated cubes or other hard-edged objects:

- Use a smaller `Max truncation`, such as `0.010 m` or `0.015 m`.
- Keep `Depth edge reject` enabled; the default is `0.020 m`.
- Use a finer turntable step, such as `5 deg`, if angular sampling artifacts are
  visible.

Real Kinect scans may need a larger truncation band because depth noise is much
higher than in simulation.

## Mesh Output

Meshes are exported as PLY files. The GUI save action lets you choose the output
path. TCP control can also save meshes with:

```json
{"type":"save_mesh","path":"/tmp/scan.ply"}
```

## Troubleshooting

- If startup fails with no CUDA device, verify `nvidia-smi` and the installed
  CUDA toolkit.
- If the GUI does not build, verify Qt5 development packages are installed or
  configure with `-DBUILD_GUI=OFF`.
- If the Kinect cannot open, verify libfreenect2 permissions and USB access.
- If simulated RGB/depth panels are black, check the Kinect pose and turntable
  geometry: the simulated object must be inside the camera frustum.
