# RobStride SDK

> **C++17 motor control library for RobStride RS-series brushless servo motors.**  
> Supports SocketCAN (Linux) and Serial (AT-command bridge). No ROS dependency.

---

## Repository Layout

```
robstride_sdk/
│
├── include/
│   └── robstride/                      # All public headers
├── src/                                # Core C++ SDK implementation
├── python/                             # pybind11 Python bindings
├── tests/                              # GTest unit and smoke tests
├── examples/
│   ├── cpp/                            # C++ Examples (Private, MIT, Serial)
│   └── python/                         # Python Examples (Private, MIT, Serial)
│
├── CMakeLists.txt                      # Standalone, no ROS required
├── package.xml                         # ament_cmake metadata (ROS workspace)
└── README.md                           # This file
```

---

## Building

### Prerequisites

```bash
sudo apt install libserial-dev libgtest-dev
pip3 install pybind11   # optional — for Python bindings
```

### Standalone Setup (Native CMake)
The SDK is completely independent of ROS and can be built natively using standard CMake. This builds the C++ library, Python bindings, and all examples.

```bash
cd robstride_sdk
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# You can now run the examples directly from the build folder:
./example_mit_operation
```

### Inside a ROS 2 Colcon Workspace
If you are integrating this SDK into a ROS 2 ecosystem (e.g., alongside `atx_hardware_v2`), you can build it using `colcon`.

```bash
cd ~/ros2_ws
colcon build --packages-select robstride_sdk
source install/setup.bash
```

---

## Using the SDK

> [!CAUTION]
> **DANGER: DRY RUN ONLY**
> The examples provided in this SDK are designed to test the motor's full dynamic capabilities (they will spin multiple rotations at high speeds). **DO NOT** run these examples if your motor is attached to a robot arm, leg, or any mechanical load. **Ensure the motor is completely free-spinning (dry mode)** before running any script to prevent catastrophic damage or injury!

> [!IMPORTANT]
> **CONFIGURE YOUR MOTOR IDs**
> The examples are hardcoded to look for Motor ID `5` and Motor ID `6` on `can0`. **Before running any example**, you MUST open the source code (e.g., `examples/cpp/mit/example_position.cpp`) and change the motor IDs to match the IDs physically configured on your motors.

Instead of cluttering this README, **fully-functional, high-frequency control examples** are provided in the `examples/` directory for both C++ and Python. 

The examples cover:
- **MIT Protocol:** High-frequency (1000Hz) impedance control (Position, Velocity, Current, CSP).
- **Private Protocol:** Parameter-based control using the motor's internal trajectory generator.
- **Serial Transport:** Running the Private protocol over a USB-to-UART bridge.

See [documentation/examples.md](documentation/examples.md) for more details, or just browse the `examples/` folder.

---

## Documentation

- **[Getting Started](documentation/getting_started.md)**: Detailed compilation and testing guide.
- **[Examples Overview](documentation/examples.md)**: Details on the differences between protocols and examples.
- **[API Reference](documentation/api_reference.md)**: Details on the `Motor` class methods.
- **[Architecture](documentation/architecture.md)**: Overview of the transport and protocol abstraction layers.
- **[Hardware Setup](documentation/hardware_setup.md)**: Guide to configuring SocketCAN or Serial bridges.

---

## Relationship to `atx_hardware_v2`

```
robstride_sdk          →   librobstride.so + pyrobstride.so
     ↑ depends on
atx_hardware_v2        →   libatx_hardware_v2.so (ros2_control plugin)
```

`atx_hardware_v2` is the ROS 2 hardware interface plugin. It links against `robstride_sdk` and provides the `CANSystem` plugin for `ros2_control`. The SDK itself has **zero ROS dependencies** and can be used in any C++17 application.
