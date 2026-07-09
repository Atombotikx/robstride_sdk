# API Reference

## `Motor` Class

The primary interface for controlling a Robstride motor.

### Constructor
```cpp
Motor(uint8_t motor_id, uint8_t host_id, Protocol protocol_type, const MotorProfile& profile, std::shared_ptr<ITransport> transport, uint32_t overrun_limit = 5);

```
- `motor_id`: The hardware ID configured on the motor (usually 1-255).
- `host_id`: The CAN ID of the controller (usually 0).
- `protocol_type`: `Protocol::PRIVATE` or `Protocol::MIT`.
- `profile`: Characteristics of the motor, such as `PROFILE_RS00`.
- `transport`: The injected transport layer (e.g., SocketCANTransport or SerialTransport).
- `overrun_limit`: Number of unacknowledged TX frames allowed before the library triggers an automatic `ERR_OVERRUN` safety shutdown (Default: 5).

### State Management
```cpp
Result enable();
Result stop();
Result clear_error();
Result set_zero();
```
- **`enable()`**: Activates the motor power stage. Must be called after `setup_mode()`.
- **`stop()`**: Disables the motor power stage and drops the motor into an idle state.
- **`clear_error()`**: Clears active fault states from the motor's memory.
- **`set_zero()`**: Sets the current physical rotor position as the new absolute 0 coordinate.

### Operational Modes
```cpp
Result setup_mode(RunMode mode, float limit1 = 0, float limit2 = 0, float limit3 = 0);
```
Configures the internal PID loops and structural limits of the motor for different control strategies. Target setpoints (like target velocity or position) are NO longer set here; they must be sent at runtime via `send_*_control()`.
- `RunMode::CURRENT` (or `TORQUE`): No limits required.
- `RunMode::SPEED`: `limit1` = Limit Cur (A), `limit2` = Accel (rad/s²)
- `RunMode::POSITION_CSP`: `limit1` = Limit Vel (rad/s)
- `RunMode::POSITION`: `limit1` = Max Vel, `limit2` = Accel, `limit3` = Decel (rad/s²)

### Real-Time Control
```cpp
Result send_torque_control(float iq_ref);
Result send_speed_control(float spd_ref);
Result send_position_control(float loc_ref);
Result send_mit_control(float position, float velocity, float kp, float kd, float torque);
```
Used to send high-frequency setpoint updates while the motor is running. These commands are "fire-and-forget" (they do not wait for an ACK) to ensure the lowest possible latency for control loops.

### Parameter I/O (Private Protocol Only)
```cpp
Result write_parameter(ParamId param, ParamValue value);
std::optional<ParamValue> read_parameter(ParamId param);
```
Reads or writes system parameters directly to the motor's volatile memory.

### Telemetry & Diagnostics
```cpp
MotorStatus get_status() const;
std::string get_status_string() const;
```
Retrieves the latest cached telemetry (angle, velocity, torque, temperature) asynchronously populated by the receive loop.

### Receive Loops
```cpp
void receive();
static void receive_all(std::shared_ptr<ITransport> transport, const std::vector<std::unique_ptr<Motor>>& motors);
```
Blocking loops designed to be run in a separate `std::thread`. `receive()` pulls data for a single motor, while `receive_all()` pulls data for an array of motors sharing a single CAN socket.

---

## `ParamValue` Struct
A union-like wrapper that safely converts floats, `uint32_t`, and `int32_t` into the raw 32-bit CAN payload required by the motor.

---

## `MotorStatus` Struct
```cpp
struct MotorStatus {
    uint8_t             motor_id;           // The ID of the motor
    uint8_t             error_mask;         // Bitmask of active faults (ERR_*)
    MotorState          state;              // Internal motor state
    RunMode             run_mode;           // Currently active control mode
    float               angle;              // Rotor angle in radians
    float               velocity;           // Rotor velocity in rad/s
    float               torque;             // Output torque in Nm
    float               temperature;        // Temperature in Celsius
    uint64_t            mcu_id;             // Hardware MCU ID (from ping)
    // ...
};
```

---

## Result Codes & Troubleshooting

When a command fails, the SDK will return a `Result` enum (in C++) or a `pyrobstride.Result` enum (in Python). If a fatal error occurs in C++, the console will print the integer representation of this enum.

| Code | Enum Name | Description & Diagnosis |
|---|---|---|
| **0** | `SUCCESS` | The command was successfully transmitted and acknowledged. |
| **1** | `ERR_TIMEOUT` | **Diagnosis:** The motor did not respond in time. Check physical wiring, CAN termination resistors (120Ω), and ensure the motor is powered on. |
| **2** | `ERR_TRANSPORT` | **Diagnosis:** The underlying OS socket or serial port threw a hardware error. Check if your CAN interface (`ifconfig can0`) or Serial port is still active and hasn't crashed. |
| **3** | `ERR_NOT_CONNECTED` | **Diagnosis:** You attempted to send a command to a motor whose transport was never `connect()`ed. |
| **4** | `ERR_PROTOCOL_MISMATCH` | **Diagnosis:** You sent an MIT command to a motor initialized as `Protocol::PRIVATE`, or vice versa. Check your `Motor` constructor. |
| **5** | `ERR_INVALID_ARGUMENT` | **Diagnosis:** You provided mathematically invalid or out-of-bounds parameters (e.g., negative torque limits). |
| **6** | `ERR_NACK` | **Diagnosis:** The motor received the command but explicitly rejected it (e.g. attempting to enable while it has an active hardware fault). Call `clear_error()` first. |
| **7** | `ERR_DISCONNECTED` | **Diagnosis:** The transport interface was disconnected or closed abruptly while the motor was trying to use it. |
| **8** | `ERR_OVERRUN` | **Diagnosis:** You are sending commands faster than the motor/bus can process them, exceeding the `overrun_limit`. Add a `sleep()` yield (e.g. 1ms) to your control loop or increase the baud rate. |
| **9** | `ERR_WRITE_FAILED` | **Diagnosis:** The transport layer failed to write bytes to the socket/port. Typically caused by a full TX buffer on the OS side (e.g., no CAN nodes are ACKing the packet). |
| **10** | `ERR_READ_FAILED` | **Diagnosis:** The transport layer failed to read bytes from the OS buffer. |
| **11** | `ERR_MALFORMED_FRAME`| **Diagnosis:** The received frame had incorrect lengths or corrupted data. This indicates heavy EMI noise on your bus lines or a baud rate mismatch. |
