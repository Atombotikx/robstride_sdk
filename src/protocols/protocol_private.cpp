#include "robstride/protocols/protocol_private.h"
#include <cstring>

namespace robstride {
namespace protocols {

bool ProtocolPrivate::is_feedback_frame(const struct can_frame& frame, uint8_t motor_id) const {
    if (!(frame.can_id & CAN_EFF_FLAG)) return false; // Private protocol uses Extended Frames
    uint32_t rx_id = frame.can_id & CAN_EFF_MASK;
    uint8_t rx_motor_id = rs_get_motor_id(rx_id);
    uint8_t msg_type    = rs_get_msg_type(rx_id);
    
    // Accept standard feedback (2), fault (21), read param (17), and device ID (0)
    FrameType type = static_cast<FrameType>(msg_type);
    return (rx_motor_id == motor_id && 
           (type == FrameType::FEEDBACK || 
            type == FrameType::FAULT || 
            type == FrameType::READ_PARAM || 
            type == FrameType::INFO));
}

bool ProtocolPrivate::parse_feedback(const struct can_frame& frame, MotorStatus& status_out, const MotorProfile& profile) const {
    uint32_t rx_id = frame.can_id & CAN_EFF_MASK;
    uint8_t msg_type = rs_get_msg_type(rx_id);
    const FrameType type = static_cast<FrameType>(msg_type);
    const uint8_t* data = frame.data;

    switch (type) {
        case FrameType::FEEDBACK: {
            status_out.state = rs_unpack_state(rx_id);
            status_out.error_mask = rs_unpack_error_mask(rx_id);

            // Extract physical states (Angle, Velocity, Torque) by unpacking bytes and applying math limits
            status_out.angle = rs_math_decode_angle(rs_unpack_uint16(data[0], data[1]));
            
            const float vel_lim = profile.velocity_limit;
            const float torq_lim = profile.torque_limit; 
            status_out.velocity = rs_math_decode_velocity(rs_unpack_uint16(data[2], data[3]), vel_lim);
            status_out.torque = rs_math_decode_torque(rs_unpack_uint16(data[4], data[5]), torq_lim);
            
            status_out.temperature = rs_math_decode_temp(rs_unpack_uint16(data[6], data[7]));
            status_out.motor_id = rs_get_motor_id(rx_id);
            break;
        }
        case FrameType::FAULT: {
            // Type 21: Fault feedback frame (Little Endian)
            status_out.fault_value = rs_unpack_uint32(data[3], data[2], data[1], data[0]);
            status_out.warning_value = rs_unpack_uint32(data[7], data[6], data[5], data[4]);
            status_out.motor_id = rs_get_motor_id(rx_id);
            break;
        }
        case FrameType::READ_PARAM: {
            // Type 17 (0x11): Read parameter response (Little Endian)
            status_out.read_param_index = rs_unpack_uint16(data[1], data[0]);
            uint32_t param_raw = rs_unpack_uint32(data[7], data[6], data[5], data[4]);
            status_out.read_param_payload = param_raw;
            status_out.motor_id = rs_get_motor_id(rx_id);
            break;
        }
        case FrameType::INFO: {
            // Type 0: Get device ID
            status_out.mcu_id = rs_unpack_uint64(data);
            status_out.motor_id = rs_get_motor_id(rx_id);
            break;
        }
        default:
            return false;
    }

    return true;
}

struct can_frame ProtocolPrivate::encode_enable(uint8_t motor_id, uint8_t host_id) const {
    return rs_hw_create_frame(rs_hw_pack_can_id(FrameType::ENABLE, motor_id, host_id));
}

struct can_frame ProtocolPrivate::encode_stop(uint8_t motor_id, uint8_t host_id) const {
    auto frame = rs_hw_create_frame(rs_hw_pack_can_id(FrameType::DISABLE, motor_id, host_id));
    frame.data[0] = RS_PAYLOAD_STOP;
    return frame;
}

struct can_frame ProtocolPrivate::encode_clear_error(uint8_t motor_id, uint8_t host_id) const {
    auto frame = rs_hw_create_frame(rs_hw_pack_can_id(FrameType::DISABLE, motor_id, host_id));
    frame.data[0] = RS_PAYLOAD_CLEAR_ERROR;
    return frame;
}

struct can_frame ProtocolPrivate::encode_ping(uint8_t motor_id, uint8_t host_id) const {
    return rs_hw_create_frame(rs_hw_pack_can_id(FrameType::INFO, motor_id, host_id));
}

struct can_frame ProtocolPrivate::encode_set_zero(uint8_t motor_id, uint8_t host_id) const {
    auto frame = rs_hw_create_frame(rs_hw_pack_can_id(FrameType::ZERO_POS, motor_id, host_id));
    frame.data[0] = RS_PAYLOAD_SET_ZERO;
    return frame;
}

struct can_frame ProtocolPrivate::encode_mit_control(uint8_t motor_id, uint8_t /*host_id*/, 
                                                     float torque, float pos, float vel, 
                                                     float kp, float kd, 
                                                     const MotorProfile& profile) const {
    
    // Scale the physical torque input into a raw 16-bit integer for the CAN ID
    uint16_t torque_raw = rs_math_encode_torque(torque, profile.torque_limit);
    auto frame = rs_hw_create_frame(rs_hw_pack_can_id(FrameType::CONTROL, motor_id, torque_raw));

    // Scale physical gains and setpoints into raw 16-bit boundaries
    uint16_t position_raw = rs_math_encode_angle(pos);
    uint16_t velocity_raw = rs_math_encode_velocity(vel, profile.velocity_limit);
    uint16_t kp_raw = rs_math_encode_kp(kp);
    uint16_t kd_raw = rs_math_encode_kd(kd);

    // Pack the raw 16-bit boundaries sequentially into the 8-byte payload using Big-Endian
    rs_hw_put_be16(&frame.data[0], position_raw);
    rs_hw_put_be16(&frame.data[2], velocity_raw);
    rs_hw_put_be16(&frame.data[4], kp_raw);
    rs_hw_put_be16(&frame.data[6], kd_raw);

    return frame;
}

struct can_frame ProtocolPrivate::encode_read_param(uint8_t motor_id, uint8_t host_id, uint16_t param) const {
    auto frame = rs_hw_create_frame(rs_hw_pack_can_id(FrameType::READ_PARAM, motor_id, host_id));
    rs_hw_put_le16(&frame.data[0], param);
    return frame;
}

struct can_frame ProtocolPrivate::encode_write_param(uint8_t motor_id, uint8_t host_id, uint16_t param, uint32_t raw_payload) const {
    auto frame = rs_hw_create_frame(rs_hw_pack_can_id(FrameType::WRITE_PARAM, motor_id, host_id));
    rs_hw_put_le16(&frame.data[0], param);
    rs_hw_put_le32(&frame.data[4], raw_payload);
    return frame;
}

FrameType ProtocolPrivate::get_response_type(const struct can_frame& frame) const {
    // Private protocol uses 29-bit Extended CAN frames.
    // The frame type is encoded in bits [31:24] of the Extended CAN ID.
    uint32_t rx_id = frame.can_id & CAN_EFF_MASK;
    return static_cast<FrameType>((rx_id >> 24) & 0xFF);
}

FrameType ProtocolPrivate::ping_response_type() const {
    // Ping sends an INFO request (type 0); the motor responds with an INFO frame
    // carrying its 64-bit MCU hardware ID in the payload.
    return FrameType::INFO;
}

} // namespace protocols
} // namespace robstride
