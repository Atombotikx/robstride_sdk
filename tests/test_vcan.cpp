#include <iostream>
#include <thread>
#include <chrono>
#include "robstride/robstride_motor.h"

int main() {
    std::cout << "Starting Robstride VCAN Test..." << std::endl;
    
    auto transport = std::make_shared<robstride::SocketCANTransport>("vcan0", 1000000); // 1 sec timeout
    if (transport->connect() != robstride::Result::SUCCESS) {
        std::cerr << "[ERROR] Failed to connect to vcan0!" << std::endl;
        std::cerr << "Did you run the vcan0 setup commands?" << std::endl;
        return -1;
    }

    // Pass the transport to the motor constructor
    robstride::Motor motor1(1, 253, robstride::Protocol::PRIVATE, robstride::PROFILE_RS00, transport);

    std::cout << "[SUCCESS] Connected to vcan0!" << std::endl;
    
    std::cout << "\n--- TEST 1: Blocking Enable ---" << std::endl;
    std::cout << "Sending enable frame and waiting 50ms for response..." << std::endl;
    
    // This will send the enable frame and block. Since there is no physical motor 
    // simulating the response on vcan0, it should elegantly timeout and return false.
    bool enabled = (motor1.enable() == robstride::Result::SUCCESS);
    
    if (enabled) {
        std::cout << "[RESULT] Enable returned TRUE! (Wait, do you have a simulator running?)" << std::endl;
    } else {
        std::cout << "[RESULT] Enable returned FALSE (Timeout expected because no physical motor is attached)" << std::endl;
    }

    std::cout << "\n--- TEST 2: Non-Blocking Fire-and-Forget Control ---" << std::endl;
    std::cout << "Sending position control (3.14 rad)..." << std::endl;
    
    // This bypasses write_parameter() and fires directly to the socket without waiting.
    bool sent = (motor1.send_position_control(3.14f) == robstride::Result::SUCCESS);
    
    std::cout << "[RESULT] Position control frame sent: " << (sent ? "TRUE" : "FALSE") << std::endl;

    std::cout << "\nTest completed. Check your 'candump vcan0' terminal to see the raw frames!" << std::endl;
    return 0;
}
