#pragma once

#include "robstride/robstride_protocol.h"
#include "robstride/robstride_types.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <linux/can.h>

namespace robstride {
namespace protocols {

// ---------------------------------------------------------
// MIT PROTOCOL MATH (12-bit & 16-bit)
// ---------------------------------------------------------

constexpr float rs_mit_math_decode_angle(uint16_t raw) {
    return raw * (25.14f / 65535.0f) - 12.57f;
}

constexpr float rs_mit_math_decode_velocity(uint16_t raw) {
    // 12-bit decode for velocity (assuming fixed +/- 33.0 limit as per manual)
    // Some motors might scale differently but the manual specifies (-33 to 33)
    return raw * (66.0f / 4095.0f) - 33.0f;
}

constexpr float rs_mit_math_decode_torque(uint16_t raw, float limit) {
    // 12-bit decode for torque using the dynamic profile limit
    return raw * (2.0f * limit / 4095.0f) - limit;
}

constexpr uint16_t rs_mit_math_encode_angle(float pos) {
    // Angle range: -4π .. +4π radians. RS_PI is defined in robstride_types.h.
    constexpr float limit = 4.0f * RS_PI;
    return (uint16_t)((std::clamp(pos, -limit, limit) + limit) * 65535.0f / (2.0f * limit));
}

constexpr uint16_t rs_mit_math_encode_velocity(float vel) {
    return (uint16_t)((std::clamp(vel, -33.0f, 33.0f) + 33.0f) * 4095.0f / 66.0f);
}

constexpr uint16_t rs_mit_math_encode_torque(float torque, float limit) {
    return (uint16_t)((std::clamp(torque, -limit, limit) + limit) * 4095.0f / (2.0f * limit));
}

constexpr uint16_t rs_mit_math_encode_kp(float kp) {
    return (uint16_t)(std::clamp(kp, 0.0f, 500.0f) * 4095.0f / 500.0f);
}

constexpr uint16_t rs_mit_math_encode_kd(float kd) {
    return (uint16_t)(std::clamp(kd, 0.0f, 5.0f) * 4095.0f / 5.0f);
}

// ---------------------------------------------------------
// MIT 12-BIT PACKING HELPERS
// ---------------------------------------------------------

// Pack four 12-bit MIT control values into a 6-byte array
constexpr void rs_hw_put_mit_12bit_control(uint8_t* ptr, uint16_t v_raw, uint16_t kp_raw, uint16_t kd_raw, uint16_t t_raw) {
    ptr[0] = v_raw >> 4;
    ptr[1] = ((v_raw & 0x0F) << 4) | (kp_raw >> 8);
    ptr[2] = kp_raw & 0xFF;
    ptr[3] = kd_raw >> 4;
    ptr[4] = ((kd_raw & 0x0F) << 4) | (t_raw >> 8);
    ptr[5] = t_raw & 0xFF;
}

constexpr uint16_t rs_unpack_mit_12bit_vel(const uint8_t* ptr) {
    return ((uint16_t)ptr[0] << 4) | (ptr[1] >> 4);
}

constexpr uint16_t rs_unpack_mit_12bit_torque(const uint8_t* ptr) {
    return (((uint16_t)ptr[0] & 0x0F) << 8) | ptr[1];
}

/**
 * @brief Strategy pattern implementation for the MIT Mini-Cheetah CAN protocol.
 *
 * ENDIANNESS POLICY:
 * - The CAN ID (11-bit standard) encapsulates mode and target ID.
 * - Multi-byte data fields in payloads (such as parameter indices) are transmitted
 *   in Little-Endian format.
 * - Position/Velocity/Torque/Kp/Kd control fields are packed into a Big-Endian layout
 *   across byte boundaries (16-bit or 12-bit packing).
 */
class ProtocolMIT : public ProtocolStrategy {
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
