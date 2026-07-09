#include "robstride/robstride_transport.h"
#include <cstring>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <poll.h>
#include <cerrno>
#include <algorithm>

namespace robstride {

// ==========================================================================
// SocketCANTransport
// ==========================================================================
SocketCANTransport::SocketCANTransport(const std::string& iface, int timeout_us)
    : iface_(iface), timeout_us_(timeout_us), fd_(-1) {}

SocketCANTransport::~SocketCANTransport() {
    disconnect();
}

Result SocketCANTransport::connect() {
    if (fd_ >= 0) return Result::SUCCESS;

    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) return Result::ERR_TRANSPORT;

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        ::close(s);
        return Result::ERR_TRANSPORT;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // Check if the interface is UP
    if (ioctl(s, SIOCGIFFLAGS, &ifr) < 0) {
        ::close(s);
        return Result::ERR_TRANSPORT;
    }
    if (!(ifr.ifr_flags & IFF_UP)) {
        ::close(s);
        return Result::ERR_NOT_CONNECTED;
    }

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ::close(s);
        return Result::ERR_TRANSPORT;
    }

    struct timeval tv{};
    tv.tv_sec  = timeout_us_ / 1000000;
    tv.tv_usec = timeout_us_ % 1000000;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof(tv));

    int loopback = 0;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK,     &loopback, sizeof(loopback));
    int recv_own_msgs = 0;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));

    fd_ = s;
    link_ok_.store(true, std::memory_order_relaxed);
    return Result::SUCCESS;
}

Result SocketCANTransport::transmit(const struct can_frame& frame) {
    if (fd_ < 0 || !link_ok_.load(std::memory_order_relaxed)) return Result::ERR_DISCONNECTED;
    std::lock_guard<std::mutex> lock(tx_mutex_);

    ssize_t nbytes = ::write(fd_, &frame, sizeof(frame));
    if (nbytes == static_cast<ssize_t>(sizeof(frame))) return Result::SUCCESS;

    const int err = errno;
    if (err == ENETDOWN || err == ENXIO || err == ENODEV || err == EBADF) {
        link_ok_.store(false, std::memory_order_relaxed);
        return Result::ERR_DISCONNECTED;
    }
    return Result::ERR_WRITE_FAILED;
}

Result SocketCANTransport::receive(struct can_frame& frame) {
    if (fd_ < 0 || !link_ok_.load(std::memory_order_relaxed)) return Result::ERR_DISCONNECTED;

    ssize_t nbytes = ::read(fd_, &frame, sizeof(frame));
    if (nbytes == static_cast<ssize_t>(sizeof(frame))) return Result::SUCCESS;

    const int err = errno;
    if (err == EAGAIN || err == EWOULDBLOCK || err == EINTR) {
        return Result::ERR_TIMEOUT;
    }
    link_ok_.store(false, std::memory_order_relaxed);
    return Result::ERR_DISCONNECTED;
}

void SocketCANTransport::disconnect() {
    link_ok_.store(false, std::memory_order_relaxed);
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SocketCANTransport::is_connected() const {
    return link_ok_.load(std::memory_order_relaxed);
}


// ==========================================================================
// SerialRingBuffer
//
// Fixed-size circular buffer. No heap. No STL containers.
// All indices are masked with (CAPACITY - 1); CAPACITY must be a power of 2.
// ==========================================================================
static_assert((SerialRingBuffer::CAPACITY & (SerialRingBuffer::CAPACITY - 1)) == 0,
              "SerialRingBuffer::CAPACITY must be a power of 2");

void SerialRingBuffer::push(const uint8_t* src, size_t len) {
    // Clamp to available space — never exceed capacity.
    const size_t space = CAPACITY - size() - 1; // keep one slot unused (full vs empty)
    if (len > space) len = space;

    for (size_t i = 0; i < len; ++i) {
        buf_[tail_ & (CAPACITY - 1)] = src[i];
        tail_ = (tail_ + 1) & (CAPACITY - 1);
    }
}

bool SerialRingBuffer::pop_frame(uint8_t* out, size_t frame_len) {
    // Minimum: need at least frame_len bytes in the buffer.
    if (size() < frame_len) return false;

    // Scan forward for a complete 'AT.....\r\n' frame.
    // We look for the \r\n terminator at position (start + frame_len - 2).
    const size_t avail = size();
    for (size_t start = 0; start + frame_len <= avail; ++start) {
        // Check AT header
        uint8_t b0 = buf_[(head_ + start)          & (CAPACITY - 1)];
        uint8_t b1 = buf_[(head_ + start + 1)      & (CAPACITY - 1)];
        uint8_t cr = buf_[(head_ + start + frame_len - 2) & (CAPACITY - 1)];
        uint8_t lf = buf_[(head_ + start + frame_len - 1) & (CAPACITY - 1)];

        if (b0 == 'A' && b1 == 'T' && cr == '\r' && lf == '\n') {
            // Discard any garbage bytes before this frame
            head_ = (head_ + start) & (CAPACITY - 1);
            // Copy frame_len bytes out
            for (size_t i = 0; i < frame_len; ++i) {
                out[i] = buf_[(head_ + i) & (CAPACITY - 1)];
            }
            // Consume from buffer
            head_ = (head_ + frame_len) & (CAPACITY - 1);
            return true;
        }
    }
    return false;
}


// ==========================================================================
// SerialTransport
// ==========================================================================
SerialTransport::SerialTransport(const std::string& port_name)
    : port_name_(port_name), fd_(-1) {}

SerialTransport::~SerialTransport() {
    disconnect();
}

Result SerialTransport::connect() {
    if (fd_ >= 0) return Result::SUCCESS;

    int s = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    if (s < 0) return Result::ERR_TRANSPORT;

    struct termios tty;
    if (tcgetattr(s, &tty) != 0) {
        ::close(s);
        return Result::ERR_TRANSPORT;
    }

    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);

    tty.c_cflag |=  (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |=  CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(s, TCSANOW, &tty) != 0) {
        ::close(s);
        return Result::ERR_TRANSPORT;
    }

    ring_buf_.reset();
    fd_ = s;
    return Result::SUCCESS;
}

Result SerialTransport::transmit(const struct can_frame& frame) {
    if (fd_ < 0) return Result::ERR_DISCONNECTED;

    // Serial transport only supports Extended (29-bit) frames.
    if (!(frame.can_id & CAN_EFF_FLAG)) return Result::ERR_PROTOCOL_MISMATCH;

    uint32_t can_id = frame.can_id & CAN_EFF_MASK;
    uint32_t at_id  = (can_id << 3) | 0x04;

    // Build the fixed 17-byte AT-frame entirely on the stack.
    uint8_t msg[17];
    msg[0] = 'A';
    msg[1] = 'T';
    msg[2] = static_cast<uint8_t>(at_id >> 24);
    msg[3] = static_cast<uint8_t>(at_id >> 16);
    msg[4] = static_cast<uint8_t>(at_id >>  8);
    msg[5] = static_cast<uint8_t>(at_id);
    msg[6] = frame.can_dlc;
    std::memcpy(&msg[7], frame.data, frame.can_dlc);
    for (int i = frame.can_dlc; i < 8; ++i) msg[7 + i] = 0;
    msg[15] = '\r';
    msg[16] = '\n';

    std::lock_guard<std::mutex> lock(tx_mutex_);
    ssize_t nbytes = ::write(fd_, msg, 17);
    return (nbytes == 17) ? Result::SUCCESS : Result::ERR_WRITE_FAILED;
}

Result SerialTransport::receive(struct can_frame& frame) {
    if (fd_ < 0) return Result::ERR_DISCONNECTED;

    // Drain whatever bytes the OS has buffered right now — no blocking, no timeout.
    // Only the RX thread calls receive(), so ring_buf_ is single-threaded here.
    {
        struct pollfd pfd{ fd_, POLLIN, 0 };
        while (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
            uint8_t tmp[64];
            ssize_t n = ::read(fd_, tmp, sizeof(tmp));
            if (n <= 0) break;
            ring_buf_.push(tmp, static_cast<size_t>(n));
        }
    }

    // Try to pop a complete 17-byte AT-frame from the ring buffer.
    constexpr size_t FRAME_LEN = 17; // AT(2) + ID(4) + DLC(1) + data(8) + \r\n(2)
    uint8_t raw[FRAME_LEN];
    if (!ring_buf_.pop_frame(raw, FRAME_LEN)) {
        return Result::ERR_TIMEOUT; // No complete frame yet — caller retries
    }

    // Parse: bytes 2..5 = big-endian 32-bit AT-ID, byte 6 = DLC, bytes 7..14 = data
    uint32_t at_id = (static_cast<uint32_t>(raw[2]) << 24)
                   | (static_cast<uint32_t>(raw[3]) << 16)
                   | (static_cast<uint32_t>(raw[4]) <<  8)
                   |  static_cast<uint32_t>(raw[5]);

    frame.can_id  = (at_id >> 3) | CAN_EFF_FLAG;
    frame.can_dlc = raw[6];
    std::memcpy(frame.data, &raw[7], 8);

    return Result::SUCCESS;
}

void SerialTransport::disconnect() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    ring_buf_.reset();
}

bool SerialTransport::is_connected() const {
    return fd_ >= 0;
}

} // namespace robstride
