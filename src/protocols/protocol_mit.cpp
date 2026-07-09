#include "robstride/protocols/protocol_mit.h"
#include <cstring>
#include <algorithm>

namespace robstride {
namespace protocols {

// Helper to create a basic 11-bit CAN frame with 8 bytes of data
static struct can_frame create_mit_frame(uint8_t motor_id, const uint8_t* payload) {
    struct can_frame frame{};
    frame.can_id = motor_id;
    frame.can_dlc = CAN_MAX_DLEN;
    if (payload) {
        std::memcpy(frame.data, payload, CAN_MAX_DLEN);
    }
    return frame;
}

bool ProtocolMIT::is_feedback_frame(const struct can_frame& frame, uint8_t motor_id) const {
    // Standard MIT uses 11-bit IDs (no EFF flag).
    if (frame.can_id & CAN_EFF_FLAG) return false;
    if (frame.can_dlc < 8) return false;
    
    uint8_t mode = (frame.can_id >> 8) & 0x07;
    uint8_t rx_id = frame.can_id & 0xFF;

    // Standard Feedback (Command 1) typically has Host ID as can_id, with Motor ID in byte 0.
    if (mode == 0 && frame.data[0] == motor_id) return true;
    
    // MCU Identification (Command 2) or Parameter Read/Write (Modes 3, 4)
    if (rx_id == motor_id) return true; 

    return false;
}

bool ProtocolMIT::parse_feedback(const struct can_frame& frame, MotorStatus& status_out, const MotorProfile& profile) const {
    uint8_t mode = (frame.can_id >> 8) & 0x07;
    uint8_t rx_id = frame.can_id & 0xFF;

    // Check if it's a standard data feedback frame (Command 1)
    if (mode == 0 && frame.data[0] == status_out.motor_id) {
        // Byte 1~2: Angle
        uint16_t angle_raw = rs_unpack_uint16(frame.data[1], frame.data[2]);
        status_out.angle = rs_mit_math_decode_angle(angle_raw);

        // Byte 3 and upper 4 bits of Byte 4: Velocity (12 bits)
        uint16_t vel_raw = rs_unpack_mit_12bit_vel(&frame.data[3]);
        status_out.velocity = rs_mit_math_decode_velocity(vel_raw);

        // Lower 4 bits of Byte 4 and Byte 5: Torque (12 bits)
        uint16_t torq_raw = rs_unpack_mit_12bit_torque(&frame.data[4]);
        status_out.torque = rs_mit_math_decode_torque(torq_raw, profile.torque_limit);

        // Byte 6~7: Temperature (Celsius * 10)
        uint16_t temp_raw = rs_unpack_uint16(frame.data[6], frame.data[7]);
        status_out.temperature = (float)temp_raw * 0.1f;
        
        status_out.state = MotorState::RUN;
        return true;
    }
    
    // Check if it's MCU Identification (Command 2)
    if (mode == 0 && rx_id == status_out.motor_id) {
        status_out.mcu_id = rs_unpack_uint64(frame.data);
        return true;
    }

    // Check if it's Read Parameter Response (Mode 3) or Write Parameter Response (Mode 4)
    if ((mode == 3 || mode == 4) && rx_id == status_out.motor_id) {
        // Little Endian parsing for parameter index
        status_out.read_param_index = rs_unpack_uint16(frame.data[1], frame.data[0]);
        // Little Endian parsing for parameter value
        uint32_t payload = rs_unpack_uint32(frame.data[7], frame.data[6], frame.data[5], frame.data[4]);
        status_out.read_param_payload = payload;
        return true;
    }
    
    return false;
}

struct can_frame ProtocolMIT::encode_enable(uint8_t motor_id, uint8_t /*host_id*/) const {
    const uint8_t payload[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    return create_mit_frame(motor_id, payload);
}

struct can_frame ProtocolMIT::encode_stop(uint8_t motor_id, uint8_t /*host_id*/) const {
    const uint8_t payload[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    return create_mit_frame(motor_id, payload);
}

struct can_frame ProtocolMIT::encode_clear_error(uint8_t motor_id, uint8_t /*host_id*/) const {
    // 0xFB -> Clear current fault. Response is Data Feedback (Command 1)
    const uint8_t payload[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    return create_mit_frame(motor_id, payload);
}

struct can_frame ProtocolMIT::encode_ping(uint8_t motor_id, uint8_t /*host_id*/) const {
    // Use a Mode-3 (read parameter) request for RUN_MODE as a side-effect-free ping.
    // The motor responds with a READ_PARAM frame, matched by ping_response_type().
    // This avoids the side effect of sending a clear-error command just to probe liveness.
    struct can_frame frame{};
    frame.can_id = (3 << 8) | motor_id;  // Mode 3: read parameter
    frame.can_dlc = CAN_MAX_DLEN;
    rs_hw_put_le16(&frame.data[0], ParamId::RUN_MODE.id);
    return frame;
}

struct can_frame ProtocolMIT::encode_set_zero(uint8_t motor_id, uint8_t /*host_id*/) const {
    const uint8_t payload[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    return create_mit_frame(motor_id, payload);
}

struct can_frame ProtocolMIT::encode_mit_control(uint8_t motor_id, uint8_t /*host_id*/, 
                                                 float torque, float pos, float vel, 
                                                 float kp, float kd, 
                                                 const MotorProfile& profile) const {
    uint16_t p_raw = rs_mit_math_encode_angle(pos);
    uint16_t v_raw = rs_mit_math_encode_velocity(vel);
    uint16_t kp_raw = rs_mit_math_encode_kp(kp);
    uint16_t kd_raw = rs_mit_math_encode_kd(kd);
    uint16_t t_raw = rs_mit_math_encode_torque(torque, profile.torque_limit);
    
    struct can_frame frame{};
    frame.can_id = motor_id;
    frame.can_dlc = CAN_MAX_DLEN;
    
    // Position is 16-bit, so we can use our Big-Endian helper
    rs_hw_put_be16(&frame.data[0], p_raw);
    
    // Velocity, Kp, Kd, and Torque are 12-bit, so they straddle byte boundaries
    // We use our centralized 12-bit packing helper for this
    rs_hw_put_mit_12bit_control(&frame.data[2], v_raw, kp_raw, kd_raw, t_raw);
    
    return frame;
}

struct can_frame ProtocolMIT::encode_read_param(uint8_t motor_id, uint8_t /*host_id*/, uint16_t param) const {
    struct can_frame frame{};
    frame.can_id = (3 << 8) | motor_id; // Mode 3
    frame.can_dlc = CAN_MAX_DLEN;
    
    rs_hw_put_le16(&frame.data[0], param);
    
    return frame;
}

struct can_frame ProtocolMIT::encode_write_param(uint8_t motor_id, uint8_t /*host_id*/, uint16_t param, uint32_t raw_payload) const {
    struct can_frame frame{};
    frame.can_id = (4 << 8) | motor_id; // Mode 4
    frame.can_dlc = CAN_MAX_DLEN;
    
    rs_hw_put_le16(&frame.data[0], param);
    
    // Little Endian packing using helpers
    rs_hw_put_le32(&frame.data[4], raw_payload);
    
    return frame;
}

FrameType ProtocolMIT::get_response_type(const struct can_frame& frame) const {
    // MIT uses 11-bit standard CAN frames; the "mode" lives in bits [10:8] of the ID.
    uint8_t mode = (frame.can_id >> 8) & 0x07;
    if (mode == 3 || mode == 4) {
        // Mode 3: read-parameter response
        // Mode 4: write-parameter response (same logical type, READ_PARAM)
        return FrameType::READ_PARAM;
    }
    // Mode 0: standard feedback frame
    return FrameType::FEEDBACK;
}

FrameType ProtocolMIT::ping_response_type() const {
    // Ping uses a Mode-3 parameter read; the motor responds with a READ_PARAM frame.
    return FrameType::READ_PARAM;
}

} // namespace protocols
} // namespace robstride
