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
#include <numeric>
#include "robstride/robstride_motor.h"

using namespace robstride;

int main() {
    std::signal(SIGINT, sigint_handler);
    std::cout << "Starting Robstride SDK Benchmark..." << std::endl;
    
    auto transport = std::make_shared<SocketCANTransport>("can0");
    if (transport->connect() != Result::SUCCESS) {
        std::cerr << "Failed to connect to can0. Ensure interface is up." << std::endl;
        return 1;
    }
    auto motor = std::make_unique<Motor>(5, 0, Protocol::PRIVATE, PROFILE_RS00, transport);
    
    // Start Background Receive Thread
    std::thread rx_thread([&motor]() {
        motor->receive();
    });

    // Let the thread spin up
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // ---------------------------------------------------------
    // Benchmark 1: Asynchronous Fire-and-Forget (send_velocity_control)
    // ---------------------------------------------------------
    std::cout << "\n--- Benchmarking Asynchronous Commands (Fire-and-Forget) ---" << std::endl;
    const int ASYNC_ITERS = 10000;
    auto start_async = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < ASYNC_ITERS; ++i) {
        motor->send_velocity_control(0.0f);
    }
    
    auto end_async = std::chrono::high_resolution_clock::now();
    double async_duration_ms = std::chrono::duration<double, std::milli>(end_async - start_async).count();
    
    std::cout << "Sent " << ASYNC_ITERS << " commands in " << async_duration_ms << " ms" << std::endl;
    std::cout << "Average time per command: " << (async_duration_ms * 1000.0 / ASYNC_ITERS) << " microseconds" << std::endl;

    // ---------------------------------------------------------
    // Benchmark 2: Synchronous Blocking Commands (ping)
    // ---------------------------------------------------------
    std::cout << "\n--- Benchmarking Synchronous Commands (Wait for ACK) ---" << std::endl;
    const int SYNC_ITERS = 100;
    std::vector<double> sync_times;
    sync_times.reserve(SYNC_ITERS);
    
    int successes = 0;
    for (int i = 0; i < SYNC_ITERS; ++i) {
        auto start_sync = std::chrono::high_resolution_clock::now();
        
        Result res = motor->ping();
        
        auto end_sync = std::chrono::high_resolution_clock::now();
        double duration_ms = std::chrono::duration<double, std::milli>(end_sync - start_sync).count();
        
        sync_times.push_back(duration_ms);
        if (res == Result::SUCCESS) successes++;
        
        // Brief pause to not overwhelm the motor during synchronous tests
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    
    double total_sync_time = std::accumulate(sync_times.begin(), sync_times.end(), 0.0);
    double avg_sync_time = total_sync_time / SYNC_ITERS;
    
    std::cout << "Success rate: " << successes << " / " << SYNC_ITERS << std::endl;
    std::cout << "Average response time (Round-Trip): " << avg_sync_time << " ms" << std::endl;
    
    // Cleanup
    motor->disconnect();
    if (rx_thread.joinable()) {
        rx_thread.join();
    }
    
    std::cout << "\nDone." << std::endl;
    return 0;
}
