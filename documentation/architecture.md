# Architecture

The Robstride SDK is built with a clear, modular architecture designed for high performance, ease of use, and multi-protocol flexibility. The SDK connects high-level motor control commands to low-level CAN bus communication.

## Core Concepts

The architecture is divided into three primary layers:
1. **The Motor (Application Layer)**
2. **The Protocol (Serialization/Deserialization Layer)**
3. **The Transport (Hardware Abstraction Layer)**

This separation of concerns follows the Dependency Injection and Strategy design patterns.

### 1. The Motor (`Motor`)
The `Motor` class is the primary interface for developers. It abstracts away all the complexities of byte-packing, protocol-specific commands, and socket communication.

- **State Management**: It maintains an internal `MotorStatus` struct which caches the latest telemetry data (angle, velocity, torque, temperature, faults).
- **Concurrency**: The `Motor` class is highly thread-safe. It uses multiple granular mutexes:
  - `command_mutex_`: Protects sequential commands (like `setup_mode()`) from being interleaved.
  - `motor_mutex_`: Protects reads and writes to the telemetry `status_` cache and coordinates the `condition_variable`.
- **Synchronous Responses**: When a command like `enable()` or `write_parameter()` is called, the Motor sends the command and uses a `std::condition_variable` to wait for a specific response frame, ensuring robust synchronous execution.

### 2. The Protocol (`ProtocolStrategy`)
The `ProtocolStrategy` is an interface that defines how data is packed into and unpacked from CAN frames. Robstride motors support two vastly different protocols over the CAN bus:
- **Private Protocol (`ProtocolPrivate`)**: Uses 29-bit Extended CAN frames. Highly featured, supports extensive parameter read/write operations, and runs up to 1 Mbps.
- **MIT Protocol (`ProtocolMIT`)**: Uses 11-bit Standard CAN frames. A lightweight, extremely fast protocol optimized for high-frequency control loops.

By injecting a protocol strategy into the `Motor` upon creation, the motor can dynamically switch how it talks to the hardware without any changes to the high-level API.

### 3. The Transport (`ITransport`)
The `ITransport` interface abstracts the physical communication medium. 
- **`SocketCANTransport`**: The primary implementation used on Linux platforms, communicating directly with the kernel's CAN subsystem (`can0`, `vcan0`).
- **Mocking**: The interface allows for easy unit testing via `MockTransport` without needing physical hardware attached.

## The Receive Loop (Threading Model)

To receive high-frequency telemetry and asynchronous responses, the SDK requires a background receive loop.

### Single Motor Setup
For simple setups, you can call `motor->receive()` in a background thread. This will block and continuously pull frames from the CAN socket, parse them, and update the internal `MotorStatus` cache.

### Multi-Motor Shared Socket Setup
A single CAN bus interface (e.g., `can0`) can only be read sequentially. If you have 3 motors on `can0`, you cannot have 3 separate threads all trying to read from `can0` simultaneously.

To solve this, the SDK provides a centralized dispatch mechanism:
```cpp
std::thread rx_thread([&]() {
    Motor::receive_all(transport, motors);
});
```
This static method loops indefinitely, reads frames from the shared transport, and dispatches each frame to every motor in the array. The `process_frame` internal method of each motor quickly checks if the frame belongs to it and parses it accordingly.

## Atomicity and Deadlock Prevention

The Robstride SDK takes careful steps to prevent deadlocks:
1. **Locking Hierarchy**: `command_mutex_` (high-level logic) -> `motor_mutex_` (state/cv) -> `transport->mutex_` (socket lock). Locks are always acquired in this order.
2. **`wait_for_response`**: When the `Motor` sends a command and waits for a response, it uses a `std::unique_lock` on `motor_mutex_` and calls `wait_for`. This *atomically releases* the mutex while sleeping, allowing the background `receive()` thread to acquire `motor_mutex_`, update the state, and trigger the `notify_all()` condition.
