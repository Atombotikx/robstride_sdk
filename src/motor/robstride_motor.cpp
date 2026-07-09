#include "robstride/robstride_motor.h"
#include "robstride/protocols/protocol_private.h"
#include "robstride/protocols/protocol_mit.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <cstring>

namespace robstride {

Motor::Motor(uint8_t motor_id, uint8_t host_id, Protocol protocol_type, const MotorProfile& profile, std::shared_ptr<ITransport> transport, uint32_t overrun_limit)
    : motor_id_(motor_id), 
      host_id_(host_id), 
      profile_(profile),
      transport_(transport),
      protocol_type_(protocol_type),
      overrun_limit_(overrun_limit)
{
    status_.motor_id = motor_id;
    status_.is_connected = false;
    
    switch (protocol_type_) {
        case Protocol::PRIVATE:
            protocol_ = std::make_unique<protocols::ProtocolPrivate>();
            break;
        case Protocol::MIT:
            protocol_ = std::make_unique<protocols::ProtocolMIT>();
            break;
        default:
            protocol_ = std::make_unique<protocols::ProtocolPrivate>();
            break;
    }
}


Motor::~Motor() {
    // Attempt to safely stop the motor before object is destroyed
    stop();
}

Result Motor::connect() {
    if (!transport_) return Result::ERR_NOT_CONNECTED;
    return transport_->connect();
}

void Motor::disconnect() {
    // Do NOT call transport_->disconnect() here, because the transport is typically
    // shared among multiple motors via Dependency Injection. Closing it would kill
    // communication for all other motors on the bus.
    status_.is_connected = false;
}

void Motor::receive() {
    if (!transport_) return;
    while (transport_->is_connected()) {
        struct can_frame frame;
        if (transport_->receive(frame) == Result::SUCCESS) {
            process_frame(frame);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void Motor::process_frame(const struct can_frame& frame) {
    if (!protocol_) return;

    if (protocol_->is_feedback_frame(frame, motor_id_)) {
        std::lock_guard<std::mutex> lock(motor_mutex_);
        
        if (protocol_->parse_feedback(frame, status_, profile_)) {
            // Automatic RX overrun matching: we received a reply!
            int prev = unacknowledged_commands_.fetch_sub(1);
            if (prev <= 0) unacknowledged_commands_.store(0); // clamp at zero
            
            // Use the protocol's own type extractor so this works for both
            // Private (29-bit extended ID encodes type in bits[31:24]) and
            // MIT (11-bit standard ID encodes mode in bits[10:8]).
            FrameType frame_type = protocol_->get_response_type(frame);
            if (frame_type == expected_response_type_) {
                response_received_ = true;
                response_cv_.notify_all();
            }
        } else {
            // Frame belonged to us but was malformed or unsupported type
            status_.unknown_frame_count++;
        }
    }
}

void Motor::process_shared_socket(std::shared_ptr<ITransport> transport, const std::vector<Motor*>& motors) {
    if (!transport || motors.empty()) return;
    
    struct can_frame frame;
    while (transport->receive(frame) == Result::SUCCESS) {
        for (const auto& motor : motors) {
            motor->process_frame(frame);
        }
    }
}

void Motor::receive_all(std::shared_ptr<ITransport> transport, const std::vector<Motor*>& motors) {
    if (!transport || motors.empty()) return;
    
    while (transport->is_connected()) {
        process_shared_socket(transport, motors);
    }
}

Result Motor::enable() {
    std::lock_guard<std::recursive_mutex> cmd_lock(command_mutex_);
    if (!protocol_ || !transport_->is_connected()) return Result::ERR_NOT_CONNECTED;
    auto frame = protocol_->encode_enable(motor_id_, host_id_);
    prepare_wait(FrameType::FEEDBACK);
    if (transport_->transmit(frame) != Result::SUCCESS) return Result::ERR_TRANSPORT;
    return wait_for_response(50);
}

Result Motor::stop() {
    std::lock_guard<std::recursive_mutex> cmd_lock(command_mutex_);
    if (!protocol_ || !transport_->is_connected()) return Result::ERR_NOT_CONNECTED;
    struct can_frame frame = protocol_->encode_stop(motor_id_, host_id_);
    prepare_wait(FrameType::FEEDBACK);
    Result tx_res = transport_->transmit(frame);
    if (tx_res != Result::SUCCESS) return tx_res;
    return wait_for_response(50);
}

Result Motor::clear_error() {
    std::lock_guard<std::recursive_mutex> cmd_lock(command_mutex_);
    if (!protocol_ || !transport_->is_connected()) return Result::ERR_NOT_CONNECTED;
    struct can_frame frame = protocol_->encode_clear_error(motor_id_, host_id_);
    prepare_wait(FrameType::FEEDBACK);
    Result tx_res = transport_->transmit(frame);
    if (tx_res != Result::SUCCESS) return tx_res;
    return wait_for_response(50);
}

Result Motor::ping() {
    std::lock_guard<std::recursive_mutex> cmd_lock(command_mutex_);
    if (!protocol_ || !transport_->is_connected()) return Result::ERR_NOT_CONNECTED;
    auto frame = protocol_->encode_ping(motor_id_, host_id_);
    // ping_response_type() is protocol-specific:
    //   Private protocol: INFO frame (type 0) — motor echoes back its MCU ID
    //   MIT protocol:     READ_PARAM frame  — response to a mode-3 parameter read
    prepare_wait(protocol_->ping_response_type());
    if (transport_->transmit(frame) != Result::SUCCESS) return Result::ERR_TRANSPORT;
    return wait_for_response(50);
}

Result Motor::set_zero() {
    std::lock_guard<std::recursive_mutex> cmd_lock(command_mutex_);
    if (!protocol_ || !transport_->is_connected()) return Result::ERR_NOT_CONNECTED;
    auto frame = protocol_->encode_set_zero(motor_id_, host_id_);
    prepare_wait(FrameType::FEEDBACK);
    if (transport_->transmit(frame) != Result::SUCCESS) return Result::ERR_TRANSPORT;
    return wait_for_response(100);
}

bool Motor::check_overrun() {
    int pending = ++unacknowledged_commands_;
    if (pending > (int)overrun_limit_) {
        unacknowledged_commands_ = overrun_limit_; // prevent integer overflow/spam
        stop(); // Safety shutdown
        return false;
    }
    return true;
}

Result Motor::send_mit_control(float torque, float pos, float vel, float kp, float kd) {
    if (!protocol_ || !transport_->is_connected()) return Result::ERR_NOT_CONNECTED;
    if (!check_overrun()) return Result::ERR_OVERRUN;
    struct can_frame frame = protocol_->encode_mit_control(motor_id_, host_id_, torque, pos, vel, kp, kd, profile_);
    return transport_->transmit(frame);
}

Result Motor::send_current_control(float iq_ref) {
    if (!protocol_ || !transport_->is_connected()) return Result::ERR_NOT_CONNECTED;
    if (!check_overrun()) return Result::ERR_OVERRUN;
    struct can_frame frame = protocol_->encode_write_param(motor_id_, host_id_, ParamId::IQ_REF.id, rs_hw_float_to_raw(iq_ref));
    return transport_->transmit(frame);
}

Result Motor::send_velocity_control(float spd_ref) {
    if (!protocol_ || !transport_->is_connected()) return Result::ERR_NOT_CONNECTED;
    if (!check_overrun()) return Result::ERR_OVERRUN;
    struct can_frame frame = protocol_->encode_write_param(motor_id_, host_id_, ParamId::SPD_REF.id, rs_hw_float_to_raw(spd_ref));
    return transport_->transmit(frame);
}

Result Motor::send_position_control(float loc_ref) {
    if (!protocol_ || !transport_->is_connected()) return Result::ERR_NOT_CONNECTED;
    if (!check_overrun()) return Result::ERR_OVERRUN;
    struct can_frame frame = protocol_->encode_write_param(motor_id_, host_id_, ParamId::LOC_REF.id, rs_hw_float_to_raw(loc_ref));
    return transport_->transmit(frame);
}

Result Motor::setup_mode(RunMode mode, float limit1, float limit2, float limit3) {
    if (write_parameter(ParamId::RUN_MODE, static_cast<uint8_t>(mode)) != Result::SUCCESS)
        return Result::ERR_NACK;

    Result params_ok = Result::SUCCESS;
    switch (mode) {
        case RunMode::OPERATION:
            break;
        case RunMode::CURRENT:
            break;
        case RunMode::SPEED:
            if ((params_ok = write_parameter(ParamId::LIMIT_CUR, limit1)) == Result::SUCCESS) {
                params_ok = write_parameter(ParamId::ACC_RAD, limit2);
            }
            break;
        case RunMode::POSITION_CSP:
            params_ok = write_parameter(ParamId::LIMIT_SPD, limit1);
            break;
        case RunMode::POSITION:
            if ((params_ok = write_parameter(ParamId::MAX_VEL, limit1)) == Result::SUCCESS &&
                (params_ok = write_parameter(ParamId::ACC_SET, limit2)) == Result::SUCCESS) {
                if (limit3 >= 0.0f) {
                    params_ok = write_parameter(ParamId::DEC_SET, limit3);
                }
            }
            break;
        default:
            return Result::ERR_INVALID_ARGUMENT;
    }
    return params_ok == Result::SUCCESS ? Result::SUCCESS : Result::ERR_NACK;
}

Result Motor::stop_pp_mode() {
    return write_parameter(ParamId::MAX_VEL, 0.0f);
}

Result Motor::write_parameter_raw(uint16_t param_id, uint32_t raw_payload) {
    std::lock_guard<std::recursive_mutex> cmd_lock(command_mutex_);
    if (!protocol_ || !transport_->is_connected()) return Result::ERR_NOT_CONNECTED;

    struct can_frame frame = protocol_->encode_write_param(motor_id_, host_id_, param_id, raw_payload);
    return transport_->transmit(frame);
}

std::optional<uint32_t> Motor::read_parameter_raw(uint16_t param_id) {
    std::lock_guard<std::recursive_mutex> cmd_lock(command_mutex_);
    if (!protocol_ || !transport_->is_connected()) return std::nullopt;

    struct can_frame frame = protocol_->encode_read_param(motor_id_, host_id_, param_id);
    prepare_wait(FrameType::READ_PARAM);
    if (transport_->transmit(frame) != Result::SUCCESS) return std::nullopt;

    if (wait_for_response(100) == Result::SUCCESS) {
        std::lock_guard<std::mutex> lock(motor_mutex_);
        if (status_.read_param_index == param_id) {
            return status_.read_param_payload;
        }
    }
    return std::nullopt;
}


void Motor::prepare_wait(FrameType expected_type) {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    expected_response_type_ = expected_type;
    response_received_ = false;
}

Result Motor::wait_for_response(int timeout_ms) {
    std::unique_lock<std::mutex> lock(motor_mutex_);
    if (response_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
        [this] { return response_received_; })) {
        return Result::SUCCESS;
    }
    return Result::ERR_TIMEOUT;
}

} // namespace robstride
