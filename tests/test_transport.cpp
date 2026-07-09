#include <gtest/gtest.h>

// Dirty trick to access private members for testing without mocking OS calls
#define private public
#include "robstride/robstride_transport.h"
#undef private

#include <cerrno>
#include <sys/socket.h>

using namespace robstride;

// Since we can't easily mock the socket system calls without dependency injection of the socket FD itself,
// we'll test the error classification logic. SocketCANTransport methods set errno and return Result.

TEST(TransportTest, ErrorClassification) {
    // This is a minimal test ensuring the constants map correctly.
    // In a real mock, we would intercept the sendto/recvfrom calls.
    
    EXPECT_EQ(static_cast<int>(Result::ERR_TIMEOUT), 1);
    EXPECT_EQ(static_cast<int>(Result::ERR_TRANSPORT), 2);
    EXPECT_EQ(static_cast<int>(Result::ERR_NOT_CONNECTED), 3);
}

TEST(TransportTest, SerialTransportEncodeDecode) {
    int fds[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);
    
    SerialTransport serial("mock");
    serial.fd_ = fds[0]; // Inject mock socket descriptor
    
    // Create a CAN frame
    struct can_frame tx_frame{};
    tx_frame.can_id = 0x01234567 | CAN_EFF_FLAG;
    tx_frame.can_dlc = 8;
    for (int i = 0; i < 8; i++) tx_frame.data[i] = i;
    
    // Transmit
    EXPECT_EQ(serial.transmit(tx_frame), Result::SUCCESS);
    
    // Read from the other end of the socket pair to verify encoding
    char buf[32];
    ssize_t nbytes = read(fds[1], buf, sizeof(buf));
    EXPECT_EQ(nbytes, 17); // "AT" + 4 bytes ID + 1 byte DLC + 8 bytes Data + \r\n
    EXPECT_EQ(buf[0], 'A');
    EXPECT_EQ(buf[1], 'T');
    EXPECT_EQ(buf[15], '\r');
    EXPECT_EQ(buf[16], '\n');
    
    // Now write a mock response to the other end
    // We'll just echo the exact same frame back
    write(fds[1], buf, 17);
    
    // Receive
    struct can_frame rx_frame{};
    EXPECT_EQ(serial.receive(rx_frame), Result::SUCCESS);
    
    // Verify decoding
    EXPECT_EQ(rx_frame.can_id, tx_frame.can_id);
    EXPECT_EQ(rx_frame.can_dlc, tx_frame.can_dlc);
    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(rx_frame.data[i], tx_frame.data[i]);
    }
    
    close(fds[0]);
    close(fds[1]);
    serial.fd_ = -1; // Prevent double close in destructor
}

