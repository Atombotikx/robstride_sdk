#pragma once

#include <stdint.h>
#include <stdbool.h>

namespace robstride {

// ---------------------------------------------------------
// MATHEMATICAL CONSTANTS
// ---------------------------------------------------------
constexpr float RS_PI = 3.14159265358979323846f;

// ---------------------------------------------------------
// ENUMS (MODERN C++11)
// ---------------------------------------------------------

/**
 * @brief Identifies the protocol variant used by the motor.
 */
enum class Protocol {
    PRIVATE = 0, ///< Robstride private protocol (Extended CAN IDs)
    MIT     = 2  ///< MIT Mini-Cheetah protocol (Standard CAN IDs)
};

/**
 * @brief Defines the operating mode of the motor.
 */
enum class RunMode {
    OPERATION    = 0, ///< Base operation mode, no active control loop
    POSITION     = 1, ///< Profile Position mode (with accel/decel limits)
    SPEED        = 2, ///< Profile Velocity mode
    CURRENT      = 3, ///< Direct Current (Torque) mode
    POSITION_CSP = 5  ///< Cyclic Synchronous Position mode (no accel profiles)
};

/**
 * @brief Indicates the current state machine phase of the motor.
 */
enum class MotorState {
    RESET       = 0, ///< Disabled or recovering from fault
    CALIBRATION = 1, ///< Currently calibrating
    RUN         = 2  ///< Active and tracking setpoints
};

// Error masks
constexpr uint8_t ERR_NONE          = 0;
constexpr uint8_t ERR_UNDERVOLTAGE  = (1 << 0);
constexpr uint8_t ERR_OVERCURRENT   = (1 << 1);
constexpr uint8_t ERR_OVERTEMP      = (1 << 2);
constexpr uint8_t ERR_ENCODER_FAULT = (1 << 3);
constexpr uint8_t ERR_OVERLOAD      = (1 << 4);
constexpr uint8_t ERR_UNCALIBRATED  = (1 << 5);

enum class Result {
    SUCCESS = 0,
    ERR_TIMEOUT = 1,
    ERR_TRANSPORT = 2,
    ERR_NOT_CONNECTED = 3,
    ERR_PROTOCOL_MISMATCH = 4,
    ERR_INVALID_ARGUMENT = 5,
    ERR_NACK = 6,
    ERR_DISCONNECTED = 7,
    ERR_OVERRUN = 8,
    ERR_WRITE_FAILED = 9,
    ERR_READ_FAILED = 10,
    ERR_MALFORMED_FRAME = 11
};

enum class FrameType : uint8_t {
    INFO        = 0,
    CONTROL     = 1,
    FEEDBACK    = 2,
    ENABLE      = 3,
    DISABLE     = 4,
    ZERO_POS    = 6,
    SET_ID      = 7,
    READ_PARAM  = 17,
    WRITE_PARAM = 18,
    FAULT       = 21,
    SAVE_PARAM  = 22,
    CHANGE_BAUD = 23
};

template <typename T>
struct TypedParam {
    uint16_t id;
};

namespace ParamId {
    constexpr TypedParam<uint8_t> RUN_MODE      = {0x7005};
    constexpr TypedParam<float>   IQ_REF        = {0x7006};
    constexpr TypedParam<float>   SPD_REF       = {0x700A};
    constexpr TypedParam<float>   LIMIT_TORQUE  = {0x700B};
    constexpr TypedParam<float>   CUR_KP        = {0x7010};
    constexpr TypedParam<float>   CUR_KI        = {0x7011};
    constexpr TypedParam<float>   CUR_FIT_GAIN  = {0x7014};
    constexpr TypedParam<float>   LOC_REF       = {0x7016};
    constexpr TypedParam<float>   LIMIT_SPD     = {0x7017};
    constexpr TypedParam<float>   LIMIT_CUR     = {0x7018};
    constexpr TypedParam<float>   MECHPOS       = {0x7019}; // Read-only usually
    constexpr TypedParam<float>   IQF           = {0x701A};
    constexpr TypedParam<float>   MECHVEL       = {0x701B};
    constexpr TypedParam<float>   VBUS          = {0x701C};
    constexpr TypedParam<float>   LOC_KP        = {0x701E};
    constexpr TypedParam<float>   SPD_KP        = {0x701F};
    constexpr TypedParam<float>   SPD_KI        = {0x7020};
    constexpr TypedParam<float>   SPD_FILT_GAIN = {0x7021};
    constexpr TypedParam<float>   ACC_RAD       = {0x7022};
    constexpr TypedParam<float>   MAX_VEL       = {0x7024};
    constexpr TypedParam<float>   ACC_SET       = {0x7025};
    constexpr TypedParam<uint16_t> EPS_CANTIME  = {0x7026};
    constexpr TypedParam<uint32_t> CAN_TIMEOUT  = {0x7028};
    constexpr TypedParam<uint8_t> ZERO_STATUS   = {0x7029};
    constexpr TypedParam<uint8_t> DAMPER        = {0x702A};
    constexpr TypedParam<float>   ADD_OFFSET    = {0x702B};
    constexpr TypedParam<uint8_t> ALVEOLOUS_OPEN= {0x702C};
    constexpr TypedParam<uint8_t> IQ_TEST       = {0x702D};
    constexpr TypedParam<float>   DEC_SET       = {0x702E};
}

// ---------------------------------------------------------
// STRUCTS
// ---------------------------------------------------------

struct MotorProfile {
    float torque_limit;
    float velocity_limit;
};

// ---------------------------------------------------------
// COMMON SERIALIZATION HELPERS
// ---------------------------------------------------------

inline uint32_t rs_hw_float_to_raw(float value) {
    uint32_t raw;
    __builtin_memcpy(&raw, &value, sizeof(float));
    return raw;
}

inline float rs_hw_raw_to_float(uint32_t raw) {
    float val;
    __builtin_memcpy(&val, &raw, sizeof(float));
    return val;
}

/**
 * @brief Represents the physical and logical state of the motor.
 * Updated automatically via background telemetry or synchronous responses.
 */
struct MotorStatus {
    uint8_t             motor_id;           ///< The configured ID of the motor
    uint8_t             error_mask;         ///< Bitmask of active faults (ERR_*)
    MotorState          state;              ///< Internal motor state
    RunMode             run_mode;           ///< Currently active control mode
    float               angle;              ///< Rotor angle in radians
    float               velocity;           ///< Rotor velocity in rad/s
    float               torque;             ///< Output torque in Nm
    float               temperature;        ///< Inverter/stator temperature in Celsius
    bool                is_connected;       ///< Transport link status
    uint8_t             host_id;            ///< Host controller ID
    uint8_t             zero_status;        ///< Zero calibration status
    uint16_t            eps_cantime;        ///< EPS CAN cycle time
    uint32_t            can_timeout;        ///< CAN timeout value
    uint64_t            mcu_id;             ///< Hardware MCU ID (from INFO frame)
    uint32_t            fault_value;        ///< Extended fault diagnostic code
    uint32_t            warning_value;      ///< Extended warning diagnostic code
    uint16_t            read_param_index;   ///< Last requested parameter ID
    uint32_t            read_param_payload; ///< Last requested parameter value
    uint32_t            unknown_frame_count;///< Number of unparseable frames received
};

// Static profile definitions
inline constexpr MotorProfile PROFILE_RS00 = {14.0f,  33.0f};
inline constexpr MotorProfile PROFILE_RS01 = {34.0f,  33.0f};
inline constexpr MotorProfile PROFILE_RS02 = {50.0f,  33.0f};
inline constexpr MotorProfile PROFILE_RS03 = {120.0f, 33.0f};

// ---------------------------------------------------------
// COMMON UNPACK/PACK
// ---------------------------------------------------------

constexpr uint16_t rs_unpack_uint16(uint8_t msb, uint8_t lsb) {
    return ((uint16_t)msb << 8) | lsb;
}

constexpr uint32_t rs_unpack_uint32(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    return ((uint32_t)b0 << 24) | ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8) | b3;
}

constexpr uint64_t rs_unpack_uint64(const uint8_t* data) {
    uint64_t val = 0;
    for (int i=0; i<8; i++) {
        val = (val << 8) | data[i];
    }
    return val;
}

constexpr void rs_hw_put_be16(uint8_t* ptr, uint16_t value) {
    ptr[0] = (uint8_t)(value >> 8);
    ptr[1] = (uint8_t)(value & 0xFF);
}

constexpr void rs_hw_put_le16(uint8_t* ptr, uint16_t value) {
    ptr[0] = (uint8_t)(value & 0xFF);
    ptr[1] = (uint8_t)(value >> 8);
}

constexpr void rs_hw_put_be32(uint8_t* ptr, uint32_t value) {
    ptr[0] = (uint8_t)(value >> 24);
    ptr[1] = (uint8_t)((value >> 16) & 0xFF);
    ptr[2] = (uint8_t)((value >> 8) & 0xFF);
    ptr[3] = (uint8_t)(value & 0xFF);
}

constexpr void rs_hw_put_le32(uint8_t* ptr, uint32_t value) {
    ptr[0] = (uint8_t)(value & 0xFF);
    ptr[1] = (uint8_t)((value >> 8) & 0xFF);
    ptr[2] = (uint8_t)((value >> 16) & 0xFF);
    ptr[3] = (uint8_t)(value >> 24);
}

inline void rs_hw_put_float(uint8_t* ptr, float value) {
    __builtin_memcpy(ptr, &value, sizeof(float));
}

} // namespace robstride
