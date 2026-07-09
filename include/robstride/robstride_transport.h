#pragma once

#include "robstride/robstride_types.h"
#include <string>
#include <linux/can.h>
#include <memory>
#include <mutex>
#include <atomic>
#include <cstdint>
#include <cstddef>

namespace robstride {

class ITransport {
public:
    virtual ~ITransport() = default;

    // Attempt to open the underlying connection.
    virtual Result connect() = 0;

    // Transmit a frame
    virtual Result transmit(const struct can_frame& frame) = 0;

    // Receive a single frame (non-blocking if none available)
    // Returns Result::SUCCESS if frame received, ERR_TIMEOUT if none.
    virtual Result receive(struct can_frame& frame) = 0;

    // Close the connection
    virtual void disconnect() = 0;

    virtual bool is_connected() const = 0;
};

class SocketCANTransport : public ITransport {
public:
    SocketCANTransport(const std::string& iface, int timeout_us = 100000);
    ~SocketCANTransport() override;

    Result connect() override;
    Result transmit(const struct can_frame& frame) override;
    Result receive(struct can_frame& frame) override;
    void disconnect() override;
    bool is_connected() const override;

private:
    std::string iface_;
    int timeout_us_;
    int fd_ = -1;
    mutable std::atomic<bool> link_ok_{false};
    std::mutex tx_mutex_;
};

// ---------------------------------------------------------------------------
// SerialRingBuffer
//
// Fixed-size, stack-allocated circular buffer for raw serial RX bytes.
// ZERO heap allocations. Thread-safety is the caller's responsibility
// (SerialTransport guards it with rx_mutex_).
// ---------------------------------------------------------------------------
class SerialRingBuffer {
public:
    static constexpr size_t CAPACITY = 512;

    void reset() { head_ = 0; tail_ = 0; }

    // Append up to 'len' bytes from 'src'. Drops oldest data if full.
    void push(const uint8_t* src, size_t len);

    // Scan for first complete AT-frame (exactly 'frame_len' bytes ending with \r\n).
    // On success, copies bytes into 'out' and consumes them. Returns true.
    bool pop_frame(uint8_t* out, size_t frame_len);

    size_t size() const {
        return (tail_ + CAPACITY - head_) % CAPACITY;
    }

private:
    uint8_t buf_[CAPACITY] = {};
    size_t  head_ = 0;   // read index
    size_t  tail_ = 0;   // write index
};

class SerialTransport : public ITransport {
public:
    explicit SerialTransport(const std::string& port_name);
    ~SerialTransport() override;

    Result connect() override;
    Result transmit(const struct can_frame& frame) override;
    Result receive(struct can_frame& frame) override;
    void disconnect() override;
    bool is_connected() const override;

private:
    std::string      port_name_;
    int              fd_ = -1;
    std::mutex       tx_mutex_;     // Guards transmit() path
    SerialRingBuffer ring_buf_;     // Only touched by RX thread — no extra mutex needed
};

} // namespace robstride
