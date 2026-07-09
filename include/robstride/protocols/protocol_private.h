#pragma once

#include "robstride/robstride_protocol.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <linux/can.h>
#include "robstride/robstride_types.h"

namespace robstride {
namespace protocols {

// ---------------------------------------------------------
// PRIVATE PROTOCOL SPECIFIC BIT-PACKING
// ---------------------------------------------------------

/**
 * @brief Composes the 29-bit Extended CAN ID required by the RobStride Private Protocol.
 * Maps the 5-bit command type, 8-bit motor ID, and optional 16-bit data area into a single ID.
 */
constexpr uint32_t rs_hw_pack_can_id(FrameType type, uint8_t motor_id, uint16_t data_area = 0) {
    return (((uint32_t)type << 24) | ((uint32_t)data_area << 8) | motor_id) | CAN_EFF_FLAG;
}

/**
 * @brief Creates a boilerplate CAN frame with standard DLC (Data Length Code) of 8 bytes.
 */
constexpr struct can_frame rs_hw_create_frame(uint32_t can_id) {
    struct can_frame frame{};
    frame.can_id = can_id;
    frame.can_dlc = CAN_MAX_DLEN; // Abstracting the hardcoded 8 bytes
    return frame;
}

constexpr uint8_t rs_get_motor_id(uint32_t can_id) {
    return (can_id >> 8) & 0xFF;
}

constexpr uint8_t rs_get_msg_type(uint32_t can_id) {
    return (can_id >> 24) & 0xFF;
}

constexpr MotorState rs_unpack_state(uint32_t rx_id) {
    return static_cast<MotorState>((rx_id >> 22) & 0x03);
}

constexpr uint8_t rs_unpack_error_mask(uint32_t rx_id) {
    return (rx_id >> 16) & 0x3F;
}

// ---------------------------------------------------------
// PRIVATE PROTOCOL PAYLOAD CONSTANTS & MATH SCALING
// ---------------------------------------------------------
// RS_PI is defined in robstride_types.h

constexpr uint8_t RS_PAYLOAD_STOP        = 0x00;
constexpr uint8_t RS_PAYLOAD_CLEAR_ERROR = 0x01;
// NOTE: RS_PAYLOAD_SET_ZERO uses the same wire value as CLEAR_ERROR (0x01)
// but is applied with FrameType::ZERO_POS, so they address different commands.
constexpr uint8_t RS_PAYLOAD_SET_ZERO    = 0x01;

// ---------------------------------------------------------
// DECODING MATH (Raw 16-bit values -> Physical Floats)
// ---------------------------------------------------------

/**
 * @brief Converts raw 16-bit payload (0 to 65535) into physical radians (-4*PI to +4*PI)
 */
constexpr float rs_math_decode_angle(uint16_t raw) {
    return raw * (8.0f * RS_PI / 65535.0f) - (4.0f * RS_PI);
}

/**
 * @brief Converts raw 16-bit payload into physical velocity (rad/s) using the motor's dynamic limit
 */
constexpr float rs_math_decode_velocity(uint16_t raw, float limit) {
    return raw * (2.0f * limit / 65535.0f) - limit;
}
/**
 * @brief Converts raw 16-bit payload into physical torque (Nm) using the motor's dynamic limit
 */
constexpr float rs_math_decode_torque(uint16_t raw, float limit) {
    return raw * (2.0f * limit / 65535.0f) - limit;
}

/**
 * @brief Converts raw 16-bit payload into a temperature float (Celsius)
 */
constexpr float rs_math_decode_temp(uint16_t raw) {
    return raw * 0.1f;
}

// ---------------------------------------------------------
// ENCODING MATH (Physical Floats -> Raw 16-bit values)
// ---------------------------------------------------------

/**
 * @brief Clamps physical torque and scales it into a 16-bit unsigned integer payload
 */
constexpr uint16_t rs_math_encode_torque(float torque, float limit) {
    return (uint16_t)((std::clamp(torque, -limit, limit) + limit) * 65535.0f / (2.0f * limit));
}

/**
 * @brief Clamps physical angle and scales it into a 16-bit unsigned integer payload
 */
constexpr uint16_t rs_math_encode_angle(float pos) {
    // Angle range: -4π .. +4π radians. RS_PI is from robstride_types.h.
    constexpr float limit = 4.0f * RS_PI;
    return (uint16_t)((std::clamp(pos, -limit, limit) + limit) * 65535.0f / (2.0f * limit));
}

/**
 * @brief Clamps physical velocity and scales it into a 16-bit unsigned integer payload
 */
constexpr uint16_t rs_math_encode_velocity(float vel, float limit) {
    return (uint16_t)((std::clamp(vel, -limit, limit) + limit) * 65535.0f / (2.0f * limit));
}

/**
 * @brief Clamps the Position Proportional Gain (Kp) into a 16-bit unsigned integer payload
 */
constexpr uint16_t rs_math_encode_kp(float kp) {
    return (uint16_t)(std::clamp(kp, 0.0f, 500.0f) * 65535.0f / 500.0f);
}

/**
 * @brief Clamps the Position Derivative Gain (Kd) into a 16-bit unsigned integer payload
 */
constexpr uint16_t rs_math_encode_kd(float kd) {
    return (uint16_t)(std::clamp(kd, 0.0f, 5.0f) * 65535.0f / 5.0f);
}

/**
 * @brief Strategy pattern implementation for the Robstride Private CAN protocol.
 *
 * ENDIANNESS POLICY:
 * - The CAN ID (29-bit extended) is constructed in Big-Endian order (Type << 24 | Data << 8 | Motor_ID).
 * - Multi-byte payload fields (like IEEE-754 floats or 32-bit/16-bit integers) are
 *   transmitted in Little-Endian byte order.
 */
class ProtocolPrivate : public ProtocolStrategy {
public:
    bool parse_feedback(const struct can_frame& frame, MotorStatus& status_out, const MotorProfile& profile) const override;

    struct can_frame encode_enable(uint8_t motor_id, uint8_t host_id) const override;
    struct can_frame encode_stop(uint8_t motor_id, uint8_t host_id) const override;
    struct can_frame encode_clear_error(uint8_t motor_id, uint8_t host_id) const override;
    struct can_frame encode_ping(uint8_t motor_id, uint8_t host_id) const override;
    struct can_frame encode_set_zero(uint8_t motor_id, uint8_t host_id) const override;
    
    struct can_frame encode_mit_control(uint8_t motor_id, uint8_t host_id, 
                                        float torque, float pos, float vel, 
                                        float kp, float kd, 
                                        const MotorProfile& profile) const override;

    struct can_frame encode_read_param(uint8_t motor_id, uint8_t host_id, uint16_t param) const override;
    struct can_frame encode_write_param(uint8_t motor_id, uint8_t host_id, uint16_t param, uint32_t raw_payload) const override;

    bool is_feedback_frame(const struct can_frame& frame, uint8_t motor_id) const override;
    FrameType get_response_type(const struct can_frame& frame) const override;
    FrameType ping_response_type() const override;

};

} // namespace protocols
} // namespace robstride
