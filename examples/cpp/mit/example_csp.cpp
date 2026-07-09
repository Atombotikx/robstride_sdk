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
    std::cout << "[" << name << "] Enabling Motor in MIT mode..." << std::endl;
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
    
    // 1. Initialize Two Motors on the shared transport for MIT protocol
    Motor motor5(5, 0, Protocol::MIT, PROFILE_RS00, transport);
    Motor motor6(6, 0, Protocol::MIT, PROFILE_RS00, transport);
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

    std::cout << "Starting High-Frequency MIT CSP Control. Press Ctrl+C to stop." << std::endl;
    
    auto start_time = std::chrono::steady_clock::now();
    
    float Kp = 50.0f; // High stiffness
    float Kd = 1.0f;  // Damping
    float last_angle5 = -999.0f;
    float last_angle6 = -999.0f;

    while (transport->is_connected() && keep_running) {
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - start_time).count();
        
        // Create a smooth sine wave from 0 to 2*PI (0 to 360 degrees)
        // Period = 4 seconds
        float target_pos = static_cast<float>(M_PI * (1.0 - std::cos(2.0 * M_PI * 0.25 * t)));
        float target_vel = static_cast<float>(M_PI * (2.0 * M_PI * 0.25) * std::sin(2.0 * M_PI * 0.25 * t));
        
        // send_mit_control(torque, pos, vel, kp, kd)
        auto res5 = motor5.send_mit_control(0.0f, target_pos, target_vel, Kp, Kd);
        if (res5 != robstride::Result::SUCCESS) {
            std::cerr << "\n\n[FATAL] Transmission failed on Motor 5 with error code: " << static_cast<int>(res5) << "! Aborting loop." << std::endl;
            break;
        }
        auto res6 = motor6.send_mit_control(0.0f, target_pos, target_vel, Kp, Kd);
        if (res6 != robstride::Result::SUCCESS) {
            std::cerr << "\n\n[FATAL] Transmission failed on Motor 6 with error code: " << static_cast<int>(res6) << "! Aborting loop." << std::endl;
            break;
        }
        
        // Read status
        auto s5 = motor5.get_status();
        auto s6 = motor6.get_status();
        if (s5.angle != last_angle5 || s6.angle != last_angle6) {
            std::cout << "\r" << std::fixed << std::setprecision(2)
                      << "M5 Tgt:" << std::setw(5) << target_pos
                      << " Act:" << std::setw(5) << s5.angle 
                      << " Vel:" << std::setw(5) << s5.velocity 
                      << " Trq:" << std::setprecision(1) << std::setw(4) << s5.torque << " | "
                      << std::setprecision(2)
                      << "M6 Tgt:" << std::setw(5) << target_pos
                      << " Act:" << std::setw(5) << s6.angle 
                      << " Vel:" << std::setw(5) << s6.velocity 
                      << " Trq:" << std::setprecision(1) << std::setw(4) << s6.torque 
                      << std::flush;
                      

            last_angle5 = s5.angle;
            last_angle6 = s6.angle;
        
        }
        
        // ~1000Hz loop
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
