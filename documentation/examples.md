# Examples Overview

The `examples/` directory contains fully functional, runnable control examples for both C++ and Python. The SDK supports two completely different protocols (Private and MIT), and the examples are split accordingly.

> [!CAUTION]
> **DANGER: DRY RUN ONLY**
> The examples provided in this SDK are designed to test the motor's full dynamic capabilities (they will spin multiple rotations at high speeds). **DO NOT** run these examples if your motor is attached to a robot arm, leg, or any mechanical load. **Ensure the motor is completely free-spinning (dry mode)** before running any script to prevent catastrophic damage or injury!

> [!IMPORTANT]
> **CONFIGURE YOUR MOTOR IDs**
> The examples are hardcoded to look for Motor ID `5` and Motor ID `6` on `can0`. **Before running any example**, you MUST open the source code (e.g., `examples/cpp/mit/example_position.cpp`) and change the motor IDs to match the IDs physically configured on your motors.

## Directory Structure

```
examples/
├── cpp/
│   ├── mit/
│   │   ├── example_csp.cpp
│   │   ├── example_current.cpp
│   │   ├── example_operation.cpp
│   │   ├── example_position.cpp
│   │   └── example_velocity.cpp
│   ├── private/
│   │   ├── example_csp.cpp
│   │   ├── example_current.cpp
│   │   ├── example_operation.cpp
│   │   ├── example_operation_serial.cpp
│   │   ├── example_position.cpp
│   │   └── example_velocity.cpp
│   └── example_benchmark.cpp
│
└── python/
    ├── mit/
    │   └── ... (same as C++)
    ├── private/
    │   └── ... (same as C++)
    └── ...
```

---

## The Protocols

### Private Protocol (Parameter Control)
The Private protocol (`examples/cpp/private/` and `examples/python/private/`) configures internal parameters on the motor. 
- You set up the mode (e.g., `SPEED` or `POSITION`) and its limits (acceleration, max velocity) using `setup_mode()`.
- The motor generates its own internal trajectory based on those limits when you send setpoints.

### MIT Protocol (High-Frequency Real-Time Control)
The MIT protocol (`examples/cpp/mit/` and `examples/python/mit/`) is designed for rapid setpoint updates.
- You do NOT configure acceleration limits internally.
- You operate exclusively in `OPERATION` mode.
- Your high-level controller (the C++ or Python script) calculates the trajectory and sends raw position, velocity, and torque targets at 500-1000 Hz using `send_mit_control()`.

---

## Running the Examples

### C++ Examples
The C++ examples are automatically compiled when you build the SDK. They are placed in the `build/` directory.

To run them (assuming you built using standard CMake):
```bash
cd build
./example_mit_csp
```

### Python Examples
The Python examples require the `pyrobstride` module to be installed or available in your `PYTHONPATH`. 

If you built the SDK via standard CMake:
```bash
# You must install the bindings to your local site-packages first.
# Using ~/.local avoids needing sudo privileges:
cd build
cmake -DCMAKE_INSTALL_PREFIX=~/.local ..
make install

# Then you can run the examples from anywhere:
python3 examples/python/mit/example_csp.py
```

If you built the SDK in a ROS 2 Colcon workspace:
```bash
# Sourcing the workspace automatically adds the library to your PYTHONPATH
source install/setup.bash
python3 src/robstride_sdk/examples/python/mit/example_csp.py
```

### Serial Transport
The Private protocol examples include `example_operation_serial`. This uses the `SerialTransport` (e.g., `/dev/ttyUSB0`) instead of SocketCAN. 
*Note: The MIT protocol does not support the Serial transport.*
