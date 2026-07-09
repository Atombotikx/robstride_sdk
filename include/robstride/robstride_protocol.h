#pragma once

#include "robstride/robstride_types.h"
#include <linux/can.h>
#include <vector>

namespace robstride {

class ProtocolStrategy {
public:
    virtual ~ProtocolStrategy() = default;

    // Decoding
    virtual bool parse_feedback(const struct can_frame& frame, MotorStatus& status_out, const MotorProfile& profile) const = 0;

    // Encoding core commands
    virtual struct can_frame encode_enable(uint8_t motor_id, uint8_t host_id) const = 0;
    virtual struct can_frame encode_stop(uint8_t motor_id, uint8_t host_id) const = 0;
    virtual struct can_frame encode_clear_error(uint8_t motor_id, uint8_t host_id) const = 0;
    virtual struct can_frame encode_ping(uint8_t motor_id, uint8_t host_id) const = 0;
    virtual struct can_frame encode_set_zero(uint8_t motor_id, uint8_t host_id) const = 0;
    
    // Asynchronous Control
    virtual struct can_frame encode_mit_control(uint8_t motor_id, uint8_t host_id, 
                                                float torque, float pos, float vel, 
                                                float kp, float kd, 
                                                const MotorProfile& profile) const = 0;

    // Parameter Read/Write
    virtual struct can_frame encode_read_param(uint8_t motor_id, uint8_t host_id, uint16_t param_id) const = 0;
    virtual struct can_frame encode_write_param(uint8_t motor_id, uint8_t host_id, uint16_t param_id, uint32_t raw_payload) const = 0;

    // Expected Response filtering
    virtual bool is_feedback_frame(const struct can_frame& frame, uint8_t motor_id) const = 0;

    /**
     * @brief Extracts the logical FrameType from a received CAN frame.
     *
     * Private protocol: type is encoded in bits [31:24] of the 29-bit Extended ID.
     * MIT protocol:     type is derived from the mode field in bits [10:8] of the 11-bit ID.
     *
     * Used by Motor::process_frame to match received frames against the expected
     * response type set by prepare_wait().
     */
    virtual FrameType get_response_type(const struct can_frame& frame) const = 0;

    /**
     * @brief Returns the FrameType the motor is expected to respond with after a ping.
     *
     * Default is FEEDBACK. Override in protocols where ping elicits a different frame
     * type (e.g. MIT uses a READ_PARAM response from a mode-3 parameter read).
     */
    virtual FrameType ping_response_type() const { return FrameType::FEEDBACK; }
};

} // namespace robstride
