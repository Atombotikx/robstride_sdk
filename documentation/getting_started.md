# Getting Started with Robstride SDK

This guide will help you install, build, and run the Robstride SDK on a Linux environment. The SDK has zero ROS dependencies, but integrates easily into a ROS 2 workspace.

## Prerequisites

- **CMake**: Version 3.10 or higher.
- **C++ Compiler**: Must support C++17.
- **Python**: Version 3.6 or higher (if building Python bindings).
- **pybind11**: For Python bindings (`sudo apt install pybind11-dev`).
- **SocketCAN**: Linux kernel CAN subsystem (built into modern Linux kernels).
- **Google Test**: If you wish to run the test suite (`sudo apt install libgtest-dev`).

---

## Building the SDK

You can build the SDK in two ways: Standalone (Native CMake) or inside a ROS 2 Workspace (Colcon).

### Method A: Standalone (Native CMake)
Use this if you are NOT using ROS 2.

1. Navigate to the SDK directory:
   ```bash
   cd robstride_sdk
   ```
2. Create a build directory and configure:
   ```bash
   mkdir build && cd build
   cmake ..
   ```
3. Compile the project:
   ```bash
   make -j$(nproc)
   ```

### Method B: ROS 2 Workspace (Colcon)
Use this if you are dropping the SDK into a ROS 2 workspace `src` folder.

1. Navigate to your workspace root:
   ```bash
   cd ~/ros2_ws
   ```
2. Build the package:
   ```bash
   colcon build --packages-select robstride_sdk
   ```
3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

---

## Running the Examples & Tests

### C++ Examples
If you built using standalone CMake, the executables are in the `build/` directory:
```bash
./build/example_mit_csp
```

If you built using `colcon`, they are in the installation bin directory, which is added to your PATH when you source the workspace. You can simply run them from anywhere:
```bash
example_mit_csp
```

### Python Examples
Python scripts are interpreted and do not need to be compiled. 

If you built standalone, you may need to tell Python where the `pyrobstride` module was compiled:
```bash
export PYTHONPATH=/path/to/robstride_sdk/build:$PYTHONPATH
python3 examples/python/mit/example_csp.py
```

If you built with `colcon` and sourced `setup.bash`, the PYTHONPATH is automatically configured for you:
```bash
python3 examples/python/mit/example_csp.py
```

### Running the Tests
To verify that the SDK compiled correctly, run the automated test suite:

```bash
# From the CMake build directory:
./test_motor
./test_protocol
./test_transport
```

---

## Virtual CAN (vcan) Testing

If you don't have physical CAN hardware connected, you can set up a virtual CAN interface (`vcan0`) to test the SDK locally:

```bash
# Load the vcan kernel module
sudo modprobe vcan

# Add a virtual CAN interface
sudo ip link add dev vcan0 type vcan

# Bring the interface up
sudo ip link set up vcan0
```
*Note: You will need to edit the examples to use `"vcan0"` instead of `"can0"`.*

---

## Next Steps

- View [Architecture](architecture.md) to understand the internal design of the SDK.
- View [API Reference](api_reference.md) for details on the classes and methods available.
- View [Examples](examples.md) to understand the difference between the MIT and Private protocols.
