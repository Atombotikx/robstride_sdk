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

bool setup_motor(Motor& motor, const std::string& name) {
    std::cout << "[" << name << "] Pinging motor..." << std::endl;
    if (motor.ping() != Result::SUCCESS) {
        std::cerr << "[" << name << "] Failed to ping motor. Is it connected and powered?" << std::endl;
        return false;
    }
    
    auto status = motor.get_status();
    std::cout << "[" << name << "] Ping successful! MCU ID: 0x" << std::hex << std::uppercase << status.mcu_id << std::dec << std::endl;
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

    std::cout << "[" << name << "] Setting up POSITION_CSP mode..." << std::endl;
    // mode, limit_spd (rad/s)
    if (motor.setup_mode(RunMode::POSITION_CSP, 10.0f) != Result::SUCCESS) {
        std::cerr << "[" << name << "] Failed to enter POSITION_CSP mode." << std::endl;
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
    std::cout << "Connecting to CAN interface..." << std::endl;
    auto transport = std::make_shared<SocketCANTransport>("can0");
    if (transport->connect() != Result::SUCCESS) {
        std::cerr << "Failed to connect to can0. Is it up?" << std::endl;
        return 1;
    }
    
    // 1. Initialize Two Motors on the shared transport
    Motor motor5(5, 0, Protocol::PRIVATE, PROFILE_RS00, transport);
    Motor motor6(6, 0, Protocol::PRIVATE, PROFILE_RS00, transport);
    std::vector<Motor*> motors = {&motor5, &motor6};

    // 2. Start shared receive thread
    std::thread rx_thread([transport, motors]() {
        Motor::receive_all(transport, motors);
    });

    // 3. Setup both motors
    if (!setup_motor(motor5, "Motor 5") || !setup_motor(motor6, "Motor 6")) {
        transport->disconnect();
        if (rx_thread.joinable()) rx_thread.join();
        return 1;
    }

    std::cout << "Starting CSP dual sweep. Press Ctrl+C to stop." << std::endl;
    
    auto start_time = std::chrono::steady_clock::now();
    float last_angle5 = -999.0f;
    float last_angle6 = -999.0f;

    while (transport->is_connected() && keep_running) {
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - start_time).count();
        
        // Smooth sine wave from 0 to 2*PI (0 to 360 degrees), Period = 4 seconds
        float target_rad = static_cast<float>(M_PI * (1.0 - std::cos(2.0 * M_PI * 0.25 * t)));
        
        // Send CSP control packet to BOTH motors and check for library-level overrun aborts
        auto res5 = motor5.send_position_control(target_rad);
        if (res5 != robstride::Result::SUCCESS) {
            std::cerr << "\n\n[FATAL] Transmission failed on Motor 5 with error code: " << static_cast<int>(res5) << "! Aborting loop." << std::endl;
            break;
        }
        auto res6 = motor6.send_position_control(target_rad);
        if (res6 != robstride::Result::SUCCESS) {
            std::cerr << "\n\n[FATAL] Transmission failed on Motor 6 with error code: " << static_cast<int>(res6) << "! Aborting loop." << std::endl;
            break;
        }
        
        // Read status
        auto s5 = motor5.get_status();
        auto s6 = motor6.get_status();
        
        // Update terminal in-place on a single line using carriage return (\r)
        if (s5.angle != last_angle5 || s6.angle != last_angle6) {
            std::cout << "\r" << std::fixed << std::setprecision(2)
                      << "M5 Tgt:" << std::setw(5) << target_rad
                      << " Act:" << std::setw(5) << s5.angle 
                      << " Vel:" << std::setw(5) << s5.velocity 
                      << " Trq:" << std::setprecision(1) << std::setw(4) << s5.torque 
                      << " Tmp:" << std::setprecision(0) << std::setw(2) << s5.temperature 
                      << " F:" << std::hex << std::uppercase << (int)s5.fault_value << std::dec << " | "
                      << "M6 Tgt:" << std::fixed << std::setprecision(2) << std::setw(5) << target_rad
                      << " Act:" << std::setw(5) << s6.angle 
                      << " Vel:" << std::setw(5) << s6.velocity 
                      << " Trq:" << std::setprecision(1) << std::setw(4) << s6.torque 
                      << " Tmp:" << std::setprecision(0) << std::setw(2) << s6.temperature 
                      << " F:" << std::hex << std::uppercase << (int)s6.fault_value << std::dec << " "
                      << std::flush;
                      

            last_angle5 = s5.angle;
            last_angle6 = s6.angle;
        
        }
        
        // Yield slightly to prevent 100% CPU lock, targeting ~1000Hz loop
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "\nStopping Motors..." << std::endl;
    motor5.stop();
    motor6.stop();
    transport->disconnect();
    
    if (rx_thread.joinable()) {
        rx_thread.join();
    }
    
    std::cout << "Done." << std::endl;
    return 0;
}
