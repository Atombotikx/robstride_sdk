#pragma once

#include "robstride/robstride_protocol.h"
#include "robstride/robstride_transport.h"
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <optional>
#include <mutex>
#include <condition_variable>

namespace robstride {

class Motor {
public:
    /**
     * @brief Construct a new Motor object
     * @param motor_id The ID of this motor on the CAN bus
     * @param host_id The ID of the host controller
     * @param protocol_type The protocol to use (PRIVATE, CANOPEN, MIT)
     * @param profile The motor limits and characteristics (e.g. PROFILE_RS00)
     * @param transport The injected transport layer
     */
    Motor(uint8_t motor_id, uint8_t host_id, Protocol protocol_type, const MotorProfile& profile, std::shared_ptr<ITransport> transport, uint32_t overrun_limit = 5);
    


    ~Motor();

    // ---------------------------------------------------------
    // TRANSPORT
    // ---------------------------------------------------------
    Result connect();
    void disconnect();
    
    /**
     * @brief Blocking receive loop for a single motor. Run in a background thread.
     * Exits automatically when the transport disconnects.
     */
    void receive();

    /**
     * @brief Writes a parameter to the motor RAM.
     */
    Result write_parameter_raw(uint16_t param_id, uint32_t raw_payload);

    /**
     * @brief Reads a parameter from the motor RAM.
     */
    std::optional<uint32_t> read_parameter_raw(uint16_t param_id);

    /**
     * @brief Writes a parameter to the motor RAM with strict compile-time type checking.
     */
    template <typename T>
    Result write_parameter(TypedParam<T> param, T value) {
        uint32_t raw = 0;
        __builtin_memcpy(&raw, &value, sizeof(T));
        return write_parameter_raw(param.id, raw);
    }

    /**
     * @brief Reads a parameter from the motor RAM with strict compile-time type checking.
     */
    template <typename T>
    std::optional<T> read_parameter(TypedParam<T> param) {
        auto raw_opt = read_parameter_raw(param.id);
        if (!raw_opt) return std::nullopt;
        T value;
        __builtin_memcpy(&value, &*raw_opt, sizeof(T));
        return value;
    }

    /**
     * @brief Blocking receive loop for multiple motors sharing one transport.
     * Dispatches each incoming frame to every motor in the array.
     * Run in a background thread.
     *
     * @param transport  The shared transport (e.g. one SocketCANTransport for can0)
     * @param motors     All motors listening on that transport
     */
    static void receive_all(std::shared_ptr<ITransport> transport,
                            const std::vector<Motor*>& motors);

    // ---------------------------------------------------------
    // COMMANDS
    // ---------------------------------------------------------
    Result enable();
    Result stop();
    Result clear_error();
    Result ping();
    Result set_zero();

    // High frequency asynchronous control
    Result send_mit_control(float torque, float pos, float vel, float kp, float kd);
    Result send_current_control(float iq_ref);
    Result send_velocity_control(float spd_ref);
    Result send_position_control(float loc_ref);

    // Synchronous Configuration Sequences
    /**
     * @brief Setup motor operation mode and structural limits
     * @param mode The RunMode to enter
     * @param limit1 First configuration limit (e.g. LIMIT_CUR, MAX_VEL, LIMIT_SPD)
     * @param limit2 Second configuration limit (e.g. ACC_RAD, ACC_SET)
     * @param limit3 Third configuration limit (e.g. DEC_SET)
     */
    Result setup_mode(RunMode mode, float limit1 = 0.0f, float limit2 = 0.0f, float limit3 = 0.0f);

    // Profile Position (PP) Special Functions
    Result stop_pp_mode();

    // Synchronous Parameter Read/Write (Blocks until response or timeout)

    // ---------------------------------------------------------
    // GETTERS
    // ---------------------------------------------------------
    MotorStatus get_status() const {
        std::lock_guard<std::mutex> lock(motor_mutex_);
        return status_;
    }
    uint8_t get_id() const { return motor_id_; }


    std::shared_ptr<ITransport> get_transport() const { return transport_; }

private:
    uint8_t motor_id_;
    uint8_t host_id_;
    MotorProfile profile_;
    MotorStatus status_{};
    mutable std::mutex motor_mutex_;

    std::recursive_mutex command_mutex_;
    std::condition_variable response_cv_;
    FrameType expected_response_type_ = FrameType::INFO;
    bool response_received_ = false;

    std::shared_ptr<ITransport> transport_;
    Protocol protocol_type_;
    std::unique_ptr<ProtocolStrategy> protocol_;
    
    // Automatic TX/RX overrun detection
    uint32_t overrun_limit_ = 5;
    std::atomic<int> unacknowledged_commands_{0};

    bool check_overrun();
    void prepare_wait(FrameType expected_type);
    Result wait_for_response(int timeout_ms = 10);

    /// Parse an incoming CAN frame and update status if it belongs to this motor.
    void process_frame(const struct can_frame& frame);

    /// Drain one batch of frames from transport and dispatch to all motors.
    static void process_shared_socket(std::shared_ptr<ITransport> transport,
                                      const std::vector<Motor*>& motors);
};

} // namespace robstride
