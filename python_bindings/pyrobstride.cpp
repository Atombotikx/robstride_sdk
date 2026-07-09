/**
 * @file pyrobstride.cpp
 * @brief Python bindings for the robstride motor driver library (Linux/SocketCAN only).
 *
 * Exposes the core C++ API to Python via pybind11.
 * Supports SocketCAN and Serial transports, the Private protocol, and
 * the full Motor command/status API.
 *
 * Usage example:
 *   import pyrobstride
 *   transport = pyrobstride.SocketCANTransport("can0")
 *   transport.connect()
 *   motor = pyrobstride.Motor(1, 0, pyrobstride.Protocol.PRIVATE, pyrobstride.PROFILE_RS00, transport)
 *   motor.enable()
 *   print(motor.get_status().angle)
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "robstride/robstride_types.h"
#include "robstride/robstride_transport.h"
#include "robstride/robstride_motor.h"

namespace py = pybind11;
using namespace robstride;

PYBIND11_MODULE(pyrobstride, m) {
    m.doc() = R"doc(
        pyrobstride - Python bindings for the RobStride Motor Driver Library
        =====================================================================

        A high-level Python interface to the robstride C++ motor control library.
        Supports SocketCAN (Linux only) and Serial transports.

        Quick Start
        -----------
        >>> import pyrobstride
        >>> t = pyrobstride.SocketCANTransport("can0")
        >>> t.connect()
        >>> motor = pyrobstride.Motor(1, 0, pyrobstride.Protocol.PRIVATE,
        ...                           pyrobstride.PROFILE_RS00, t)
        >>> motor.enable()
        >>> status = motor.get_status()
        >>> print(f"Angle: {status.angle:.3f} rad, Vel: {status.velocity:.3f} rad/s")
    )doc";

    // ---------------------------------------------------------
    // Enums
    // ---------------------------------------------------------
    py::enum_<Protocol>(m, "Protocol", "Motor communication protocol selection.")
        .value("PRIVATE", Protocol::PRIVATE, "RobStride Private CAN protocol (supported over both CAN and Serial).")
        .value("MIT",     Protocol::MIT,     "MIT mini-cheetah CAN protocol (SocketCAN only).")
        .export_values();

    py::enum_<Result>(m, "Result", "Return codes from all motor operations.")
        .value("SUCCESS",              Result::SUCCESS)
        .value("ERR_TIMEOUT",          Result::ERR_TIMEOUT)
        .value("ERR_TRANSPORT",        Result::ERR_TRANSPORT)
        .value("ERR_NOT_CONNECTED",    Result::ERR_NOT_CONNECTED)
        .value("ERR_PROTOCOL_MISMATCH",Result::ERR_PROTOCOL_MISMATCH)
        .value("ERR_INVALID_ARGUMENT", Result::ERR_INVALID_ARGUMENT)
        .value("ERR_NACK",             Result::ERR_NACK)
        .value("ERR_OVERRUN",          Result::ERR_OVERRUN)
        .value("ERR_DISCONNECTED",     Result::ERR_DISCONNECTED)
        .value("ERR_WRITE_FAILED",     Result::ERR_WRITE_FAILED)
        .value("ERR_READ_FAILED",      Result::ERR_READ_FAILED)
        .value("ERR_MALFORMED_FRAME",  Result::ERR_MALFORMED_FRAME)
        .export_values();

    py::enum_<RunMode>(m, "RunMode", "Motor operational mode for parameter-based control.")
        .value("OPERATION",    RunMode::OPERATION)
        .value("POSITION",     RunMode::POSITION)
        .value("SPEED",        RunMode::SPEED)
        .value("CURRENT",      RunMode::CURRENT)
        .value("POSITION_CSP", RunMode::POSITION_CSP)
        .export_values();

    py::enum_<MotorState>(m, "MotorState", "Current operational state reported by the motor.")
        .value("RESET",       MotorState::RESET)
        .value("CALIBRATION", MotorState::CALIBRATION)
        .value("RUN",         MotorState::RUN)
        .export_values();

    py::class_<TypedParam<uint8_t>>(m, "TypedParam_uint8").def_readwrite("id", &TypedParam<uint8_t>::id);
    py::class_<TypedParam<uint16_t>>(m, "TypedParam_uint16").def_readwrite("id", &TypedParam<uint16_t>::id);
    py::class_<TypedParam<uint32_t>>(m, "TypedParam_uint32").def_readwrite("id", &TypedParam<uint32_t>::id);
    py::class_<TypedParam<float>>(m, "TypedParam_float").def_readwrite("id", &TypedParam<float>::id);

    auto param_id = m.def_submodule("ParamId", "Motor register parameter IDs.");
    param_id.attr("RUN_MODE")      = ParamId::RUN_MODE;
    param_id.attr("IQ_REF")        = ParamId::IQ_REF;
    param_id.attr("SPD_REF")       = ParamId::SPD_REF;
    param_id.attr("LIMIT_TORQUE")  = ParamId::LIMIT_TORQUE;
    param_id.attr("CUR_KP")        = ParamId::CUR_KP;
    param_id.attr("CUR_KI")        = ParamId::CUR_KI;
    param_id.attr("CUR_FIT_GAIN")  = ParamId::CUR_FIT_GAIN;
    param_id.attr("LOC_REF")       = ParamId::LOC_REF;
    param_id.attr("LIMIT_SPD")     = ParamId::LIMIT_SPD;
    param_id.attr("LIMIT_CUR")     = ParamId::LIMIT_CUR;
    param_id.attr("MECHPOS")       = ParamId::MECHPOS;
    param_id.attr("IQF")           = ParamId::IQF;
    param_id.attr("MECHVEL")       = ParamId::MECHVEL;
    param_id.attr("VBUS")          = ParamId::VBUS;
    param_id.attr("LOC_KP")        = ParamId::LOC_KP;
    param_id.attr("SPD_KP")        = ParamId::SPD_KP;
    param_id.attr("SPD_KI")        = ParamId::SPD_KI;
    param_id.attr("SPD_FILT_GAIN") = ParamId::SPD_FILT_GAIN;
    param_id.attr("ACC_RAD")       = ParamId::ACC_RAD;
    param_id.attr("MAX_VEL")       = ParamId::MAX_VEL;
    param_id.attr("ACC_SET")       = ParamId::ACC_SET;
    param_id.attr("EPS_CANTIME")   = ParamId::EPS_CANTIME;
    param_id.attr("CAN_TIMEOUT")   = ParamId::CAN_TIMEOUT;
    param_id.attr("ZERO_STATUS")   = ParamId::ZERO_STATUS;
    param_id.attr("DAMPER")        = ParamId::DAMPER;
    param_id.attr("ADD_OFFSET")    = ParamId::ADD_OFFSET;
    param_id.attr("ALVEOLOUS_OPEN")= ParamId::ALVEOLOUS_OPEN;
    param_id.attr("IQ_TEST")       = ParamId::IQ_TEST;
    param_id.attr("DEC_SET")       = ParamId::DEC_SET;

    // ---------------------------------------------------------
    // Structs
    // ---------------------------------------------------------
    py::class_<MotorProfile>(m, "MotorProfile",
        "Physical limits profile for a specific motor model.")
        .def(py::init<>())
        .def(py::init([](float torque, float vel){
            MotorProfile p; p.torque_limit = torque; p.velocity_limit = vel; return p;
        }), py::arg("torque_limit"), py::arg("velocity_limit"),
            "Construct a motor profile with given torque and velocity limits.")
        .def_readwrite("torque_limit",   &MotorProfile::torque_limit,   "Peak torque limit (Nm).")
        .def_readwrite("velocity_limit", &MotorProfile::velocity_limit, "Peak velocity limit (rad/s).")
        .def("__repr__", [](const MotorProfile& p){
            return "<MotorProfile torque_limit=" + std::to_string(p.torque_limit) +
                   " velocity_limit=" + std::to_string(p.velocity_limit) + ">";
        });

    py::class_<MotorStatus>(m, "MotorStatus",
        "Snapshot of the motor's complete reported state.")
        .def(py::init<>())
        .def_readwrite("motor_id",         &MotorStatus::motor_id,         "Motor CAN ID.")
        .def_readwrite("error_mask",       &MotorStatus::error_mask,       "Bitmask of active faults.")
        .def_readwrite("state",            &MotorStatus::state,            "Operational state (RESET/CALIBRATION/RUN).")
        .def_readwrite("run_mode",         &MotorStatus::run_mode,         "Current run mode.")
        .def_readwrite("angle",            &MotorStatus::angle,            "Shaft angle (radians).")
        .def_readwrite("velocity",         &MotorStatus::velocity,         "Shaft velocity (rad/s).")
        .def_readwrite("torque",           &MotorStatus::torque,           "Shaft torque (Nm).")
        .def_readwrite("temperature",      &MotorStatus::temperature,      "Driver temperature (°C).")
        .def_readwrite("is_connected",     &MotorStatus::is_connected,     "Transport connection state.")
        .def_readwrite("host_id",          &MotorStatus::host_id,          "Host controller CAN ID.")
        .def_readwrite("mcu_id",           &MotorStatus::mcu_id,           "Unique MCU hardware ID.")
        .def_readwrite("fault_value",      &MotorStatus::fault_value,      "Raw fault register value.")
        .def_readwrite("warning_value",    &MotorStatus::warning_value,    "Raw warning register value.")
        .def_readwrite("read_param_index", &MotorStatus::read_param_index, "Last read parameter ID.")
        .def_readwrite("read_param_payload", &MotorStatus::read_param_payload, "Last read parameter payload.")
        .def("__repr__", [](const MotorStatus& s){
            return "<MotorStatus id=" + std::to_string(s.motor_id) +
                   " angle=" + std::to_string(s.angle) +
                   " vel=" + std::to_string(s.velocity) +
                   " torque=" + std::to_string(s.torque) + ">";
        });

    // ---------------------------------------------------------
    // Transport: ITransport interface (base class for Python)
    // ---------------------------------------------------------
    py::class_<ITransport, std::shared_ptr<ITransport>>(m, "ITransport",
        "Abstract base class for motor communication transports.")
        .def("connect",      &ITransport::connect,       "Open the transport connection.")
        .def("disconnect",   &ITransport::disconnect,    "Close the transport connection.")
        .def("is_connected", &ITransport::is_connected,  "Check if the transport is open.");

    // SocketCAN transport
    py::class_<SocketCANTransport, ITransport, std::shared_ptr<SocketCANTransport>>(
        m, "SocketCANTransport",
        R"doc(
        SocketCAN transport for Linux-native CAN bus.

        Requires a SocketCAN interface (e.g. ``can0``) to be configured
        before use::

            sudo ip link set can0 type can bitrate 1000000
            sudo ip link set can0 up

        Supports all protocols (PRIVATE and MIT).
        )doc")
        .def(py::init<const std::string&>(), py::arg("interface"),
             "Construct a SocketCAN transport bound to ``interface`` (e.g. ``can0``).")
        .def("connect",      &SocketCANTransport::connect,      "Open the SocketCAN socket.")
        .def("disconnect",   &SocketCANTransport::disconnect,   "Close the SocketCAN socket.")
        .def("is_connected", &SocketCANTransport::is_connected, "True if the socket is open.");

    // Serial transport (Private protocol only)
    py::class_<SerialTransport, ITransport, std::shared_ptr<SerialTransport>>(
        m, "SerialTransport",
        R"doc(
        Serial/UART transport for the AT-command bridge.

        **Private protocol only.** Attempting to use MIT protocol with this
        transport will throw a ``ValueError`` at construction time.

        Baud rate is fixed at 921600. Requires the device to be accessible
        and the calling user to have read/write permission on the port.

        Example::

            t = pyrobstride.SerialTransport("/dev/ttyUSB0")
            t.connect()
        )doc")
        .def(py::init<const std::string&>(), py::arg("port"),
             "Construct a Serial transport bound to the given port (e.g. ``/dev/ttyUSB0``).")
        .def("connect",      &SerialTransport::connect,      "Open and configure the serial port at 921600 baud.")
        .def("disconnect",   &SerialTransport::disconnect,   "Close the serial port.")
        .def("is_connected", &SerialTransport::is_connected, "True if the port is open.");

    // ---------------------------------------------------------
    // Motor - pre-defined profiles as module-level constants
    // ---------------------------------------------------------
    m.attr("PROFILE_RS00") = PROFILE_RS00;
    m.attr("PROFILE_RS01") = PROFILE_RS01;
    m.attr("PROFILE_RS02") = PROFILE_RS02;
    m.attr("PROFILE_RS03") = PROFILE_RS03;

    // ---------------------------------------------------------
    // Motor class
    // ---------------------------------------------------------
    py::class_<Motor>(m, "Motor",
        R"doc(
        A single RobStride motor controller instance.

        The ``Motor`` owns no transport; it receives a shared transport that may be
        reused across multiple motors on the same bus. All command methods are
        serialized internally with a mutex, making them safe to call from any thread.

        Args:
            motor_id:  CAN ID of the target motor (1-127).
            host_id:   CAN ID of the host controller (typically 0).
            protocol:  Protocol variant (PRIVATE or MIT).
            profile:   Physical limits profile for this motor model.
            transport: Opened transport instance shared by all motors on the bus.

        Raises:
            RuntimeError: If a SerialTransport is passed with a non-PRIVATE protocol.
        )doc")
        .def(py::init<uint8_t, uint8_t, Protocol, MotorProfile, std::shared_ptr<ITransport>, uint32_t>(),
             py::arg("motor_id"), py::arg("host_id"),
             py::arg("protocol"), py::arg("profile"), py::arg("transport"), py::arg("overrun_limit") = 5)


        // Connection
        .def("connect",      &Motor::connect,      py::call_guard<py::gil_scoped_release>(), "Connect the motor's internal transport.")
        .def("disconnect",   &Motor::disconnect,   py::call_guard<py::gil_scoped_release>(), "Disconnect the motor's internal transport.")
        .def("receive",      &Motor::receive,      py::call_guard<py::gil_scoped_release>(), "Block and continuously process incoming frames for this motor. Call in a background thread.")

        // Core commands
        .def("enable",       &Motor::enable,       py::call_guard<py::gil_scoped_release>(), "Enable the motor (exits RESET, enters RUN). Blocks until ACK.")
        .def("stop",         &Motor::stop,         py::call_guard<py::gil_scoped_release>(), "Disable torque output. Motor enters RESET state. Blocks until ACK.")
        .def("clear_error",  &Motor::clear_error,  py::call_guard<py::gil_scoped_release>(), "Clear latched fault flags. Blocks until ACK.")
        .def("set_zero",     &Motor::set_zero,     py::call_guard<py::gil_scoped_release>(), "Set current position as the zero reference. Blocks until ACK.")
        .def("ping",         &Motor::ping,         py::call_guard<py::gil_scoped_release>(), "Ping the motor. Returns SUCCESS if the motor is responding.")

        // Real-time control (fire-and-forget, no blocking)
        .def("send_mit_control", &Motor::send_mit_control,
             py::arg("torque"), py::arg("pos"), py::arg("vel"), py::arg("kp"), py::arg("kd"),
             "Send MIT-mode torque+position+velocity command. Non-blocking.")
        .def("send_current_control",  &Motor::send_current_control,
             py::arg("iq_ref"), "Set direct current reference (A). Non-blocking.")
        .def("send_velocity_control", &Motor::send_velocity_control,
             py::arg("spd_ref"), "Set velocity setpoint (rad/s). Non-blocking.")
        .def("send_position_control", &Motor::send_position_control,
             py::arg("loc_ref"), "Set position setpoint (rad). Non-blocking.")

        // Write parameter overloads
        .def("write_parameter", [](Motor& self, TypedParam<float> param, float val) { return self.write_parameter(param, val); }, py::arg("param"), py::arg("value"))
        .def("write_parameter", [](Motor& self, TypedParam<uint32_t> param, uint32_t val) { return self.write_parameter(param, val); }, py::arg("param"), py::arg("value"))
        .def("write_parameter", [](Motor& self, TypedParam<uint16_t> param, uint16_t val) { return self.write_parameter(param, val); }, py::arg("param"), py::arg("value"))
        .def("write_parameter", [](Motor& self, TypedParam<uint8_t> param, uint8_t val) { return self.write_parameter(param, val); }, py::arg("param"), py::arg("value"))
        
        // Read parameter overloads
        .def("read_parameter", [](Motor& self, TypedParam<float> param) { return self.read_parameter(param); }, py::arg("param"))
        .def("read_parameter", [](Motor& self, TypedParam<uint32_t> param) { return self.read_parameter(param); }, py::arg("param"))
        .def("read_parameter", [](Motor& self, TypedParam<uint16_t> param) { return self.read_parameter(param); }, py::arg("param"))
        .def("read_parameter", [](Motor& self, TypedParam<uint8_t> param) { return self.read_parameter(param); }, py::arg("param"))

        // Setup
        .def("setup_mode", &Motor::setup_mode,
             py::arg("mode"),
             py::arg("limit1") = 0.0f,
             py::arg("limit2") = 0.0f,
             py::arg("limit3") = 0.0f,
             py::call_guard<py::gil_scoped_release>(),
             "Configure motor run mode and structural limits. limit1-limit3 are mode-specific parameters.")

        // Status accessors
        .def("get_status",   &Motor::get_status,   "Return a snapshot copy of the latest motor status.")
        .def("get_id",       &Motor::get_id,       "Return the motor CAN ID.")
        .def("get_transport",&Motor::get_transport,"Return the shared transport instance.")

        .def_static("receive_all", &Motor::receive_all,
             py::arg("transport"), py::arg("motors"), py::call_guard<py::gil_scoped_release>(),
             "Block and process incoming frames for an array of motors on a shared transport.");

    // ---------------------------------------------------------
    // Error mask bit-flag constants
    // ---------------------------------------------------------
    m.attr("ERR_NONE")          = ERR_NONE;
    m.attr("ERR_UNDERVOLTAGE")  = ERR_UNDERVOLTAGE;
    m.attr("ERR_OVERCURRENT")   = ERR_OVERCURRENT;
    m.attr("ERR_OVERTEMP")      = ERR_OVERTEMP;
    m.attr("ERR_ENCODER_FAULT") = ERR_ENCODER_FAULT;
    m.attr("ERR_OVERLOAD")      = ERR_OVERLOAD;
    m.attr("ERR_UNCALIBRATED")  = ERR_UNCALIBRATED;
}
