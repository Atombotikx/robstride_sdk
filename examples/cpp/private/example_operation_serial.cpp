/**
 * example_operation_serial.cpp — Private protocol operation mode over Serial (UART).
 *
 * Hardware setup:
 *   - Motor connected via USB-to-UART adapter (default: /dev/ttyUSB0, 921600 baud)
 *   - Motor IDs: 5 and 6
 *   - Protocol: PRIVATE
 *   - Change SERIAL_PORT below to match your actual device node.
 */
#include <iostream>
#include <csignal>
#include <atomic>

std::atomic<bool> keep_running{true};

void sigint_handler(int) {
    keep_running = false;
}

#include <thread>
#include <vector>
#include <chrono>
#include <cmath>
#include <iomanip>
#include "robstride/robstride_motor.h"
#include "robstride/robstride_transport.h"

using namespace robstride;

static const std::string SERIAL_PORT = "/dev/ttyUSB0"; // Change to your device

bool setup_motor(Motor& motor, const std::string& name) {
    std::cout << "[" << name << "] Pinging motor..." << std::endl;
    if (motor.ping() != Result::SUCCESS) {
        std::cerr << "[" << name << "] Failed to ping motor. Is it connected and powered?" << std::endl;
        return false;
    }

    auto status = motor.get_status();
    std::cout << "[" << name << "] Ping successful! MCU ID: 0x"
              << std::hex << std::uppercase << status.mcu_id << std::dec << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "[" << name << "] Clearing errors..." << std::endl;
    if (motor.clear_error() != Result::SUCCESS) {
        std::cerr << "[" << name << "] Failed to clear errors." << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "[" << name << "] Setting current position as absolute zero..." << std::endl;
    if (motor.set_zero() != Result::SUCCESS) {
        std::cerr << "[" << name << "] Failed to set zero position." << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "[" << name << "] Setting up OPERATION mode..." << std::endl;
    if (motor.setup_mode(RunMode::OPERATION) != Result::SUCCESS) {
        std::cerr << "[" << name << "] Failed to enter OPERATION mode." << std::endl;
        return false;
    }

    std::cout << "[" << name << "] Enabling Motor..." << std::endl;
    if (motor.enable() != Result::SUCCESS) {
        std::cerr << "[" << name << "] Failed to enable motor." << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return true;
}

int main() {
    std::signal(SIGINT, sigint_handler);

    std::cout << "Connecting to Serial interface on " << SERIAL_PORT << "..." << std::endl;
    auto transport = std::make_shared<SerialTransport>(SERIAL_PORT);
    if (transport->connect() != Result::SUCCESS) {
        std::cerr << "Failed to connect to " << SERIAL_PORT << "." << std::endl;
        std::cerr << "Is it plugged in? Try: sudo chmod 666 " << SERIAL_PORT << std::endl;
        return 1;
    }

    // Initialize ONE Motor on the shared serial transport
    Motor motor5(5, 0, Protocol::PRIVATE, PROFILE_RS00, transport);
    std::vector<Motor*> motors = {&motor5};

    // Start shared receive thread
    std::thread rx_thread([transport, motors]() {
        Motor::receive_all(transport, motors);
    });

    // Setup motor
    if (!setup_motor(motor5, "Motor 5")) {
        transport->disconnect();
        if (rx_thread.joinable()) rx_thread.join();
        return 1;
    }

    std::cout << "Starting high-frequency sweep over Serial. Press Ctrl+C to stop." << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    float Kp = 5.0f;
    float Kd = 0.5f;



    // NOTE: Serial has lower bandwidth than CAN. One motor at 1000Hz is well within
    // the 921600 baud limit.
    float last_angle5 = -999.0f;

    while (transport->is_connected() && keep_running) {
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - start_time).count();

        // Smooth sine wave from 0 to 2*PI (0 to 360 degrees), Period = 4 seconds
        float target_rad = static_cast<float>(M_PI * (1.0 - std::cos(2.0 * M_PI * 0.25 * t)));

        auto res = motor5.send_mit_control(0.0f, target_rad, 0.0f, Kp, Kd);
        if (res != robstride::Result::SUCCESS) {
            std::cerr << "\n\n[FATAL] Transmission failed with error code: " << static_cast<int>(res) << "! Aborting loop." << std::endl;
            break;
        }

        auto s5 = motor5.get_status();
        if (s5.angle != last_angle5) {
            std::cout << "\r" << std::fixed << std::setprecision(2)
                      << "M5 Act:" << std::setw(5) << s5.angle
                      << " Vel:" << std::setw(5) << s5.velocity
                      << " Trq:" << std::setprecision(1) << std::setw(4) << s5.torque
                      << " Tmp:" << std::setprecision(0) << std::setw(2) << s5.temperature
                      << " F:" << std::hex << std::uppercase << (int)s5.fault_value << std::dec << "    "
                      << std::flush;


            last_angle5 = s5.angle;
        
        }

        // 1ms yield
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "\nStopping Motor..." << std::endl;
    motor5.stop();
    transport->disconnect();

    if (rx_thread.joinable()) {
        rx_thread.join();
    }

    std::cout << "Done." << std::endl;
    return 0;
}
