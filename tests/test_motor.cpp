#include <gtest/gtest.h>
#include "robstride/robstride_motor.h"
#include "robstride/robstride_transport.h"
#include <vector>
#include <thread>

using namespace robstride;

class MockTransport : public ITransport {
public:
    Result connect() override { return Result::SUCCESS; }
    
    Result transmit(const struct can_frame& frame) override {
        std::lock_guard<std::mutex> lock(mutex_);
        last_tx_frame = frame;
        tx_count++;
        
        // Auto-respond for the test
        // TX enable frame layout: (ENABLE=3 << 24) | (data_area=0 << 8) | motor_id=1
        //   = 0x03000001 | CAN_EFF_FLAG
        // RX feedback frame layout: (FEEDBACK=2 << 24) | (state=RUN=2 << 22) | (motor_id=1 << 8) | host=0
        //   = 0x02800100 | CAN_EFF_FLAG
        struct can_frame response{};
        uint32_t tx_raw = frame.can_id & CAN_EFF_MASK;
        uint8_t tx_type = (tx_raw >> 24) & 0xFF;
        if (tx_type == 3) { // ENABLE command
            uint8_t motor_id = tx_raw & 0xFF;
            response.can_id = (0x02000000 | (2u << 22) | (uint32_t(motor_id) << 8) | 0) | CAN_EFF_FLAG;
            response.can_dlc = 8;
            rx_queue.push_back(response);
        } else if (tx_type == 4) { // STOP command
            uint8_t motor_id = tx_raw & 0xFF;
            response.can_id = (0x02000000 | (0u << 22) | (uint32_t(motor_id) << 8) | 0) | CAN_EFF_FLAG;
            response.can_dlc = 8;
            rx_queue.push_back(response);
        } else if (tx_type == 6) { // SET_ZERO
            uint8_t motor_id = tx_raw & 0xFF;
            response.can_id = (0x02000000 | (2u << 22) | (uint32_t(motor_id) << 8) | 0) | CAN_EFF_FLAG;
            response.can_dlc = 8;
            rx_queue.push_back(response);
        } else if (tx_type == 18) { // WRITE_PARAM
            uint8_t motor_id = tx_raw & 0xFF;
            uint16_t param = frame.data[0] | (frame.data[1] << 8);
            uint32_t val = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24);
            
            params_db_[param] = val;
        } else if (tx_type == 17) { // READ_PARAM
            uint8_t motor_id = tx_raw & 0xFF;
            uint16_t param = frame.data[0] | (frame.data[1] << 8);
            response.can_id = (17u << 24) | (uint32_t(motor_id) << 8) | CAN_EFF_FLAG;
            response.can_dlc = 8;
            response.data[0] = frame.data[0];
            response.data[1] = frame.data[1];
            if (param == ParamId::CAN_TIMEOUT.id) {
                return Result::SUCCESS;
            }
            uint32_t val = params_db_[param];
            response.data[4] = val & 0xFF;
            response.data[5] = (val >> 8) & 0xFF;
            response.data[6] = (val >> 16) & 0xFF;
            response.data[7] = (val >> 24) & 0xFF;
            rx_queue.push_back(response);
        }
        
        return Result::SUCCESS;
    }
    
    Result receive(struct can_frame& frame) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!rx_queue.empty()) {
            frame = rx_queue.front();
            rx_queue.erase(rx_queue.begin());
            return Result::SUCCESS;
        }
        return Result::ERR_TIMEOUT;
    }
    
    void disconnect() override { connected_ = false; }
    bool is_connected() const override { return connected_; }
    
    struct can_frame last_tx_frame{};
    int tx_count = 0;
    std::vector<struct can_frame> rx_queue;
    std::mutex mutex_;
    std::map<uint16_t, uint32_t> params_db_;
    std::atomic<bool> connected_{true};
};

class MotorTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_transport = std::make_shared<MockTransport>();
        motor = std::make_unique<Motor>(1, 0, Protocol::PRIVATE, PROFILE_RS00, mock_transport);
        rx_thread_ = std::thread([this]() {
            motor->receive();
        });
        // Settle time for rx_thread
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    void TearDown() override {
        mock_transport->disconnect();
        if (rx_thread_.joinable()) {
            rx_thread_.join();
        }
    }

    std::shared_ptr<MockTransport> mock_transport;
    std::unique_ptr<Motor> motor;
    std::thread rx_thread_;
};

TEST_F(MotorTest, EnableSendsCorrectFrameAndWaits) {
    // The MockTransport will automatically respond when it receives the enable frame.
    Result result = motor->enable();
    EXPECT_EQ(result, Result::SUCCESS);
    EXPECT_EQ(motor->get_status().state, MotorState::RUN);

    // Verify what it transmitted
    EXPECT_EQ(mock_transport->tx_count, 1);
    // enable TX: (ENABLE=3 << 24) | (data_area=0 << 8) | motor_id=1 | CAN_EFF_FLAG
    EXPECT_EQ(mock_transport->last_tx_frame.can_id & CAN_EFF_MASK, 0x03000001u);
}

TEST_F(MotorTest, ReadParameterTimeout) {
    // We configured MockTransport to timeout on ParamId::CAN_TIMEOUT
    auto val = motor->read_parameter(ParamId::CAN_TIMEOUT);
    EXPECT_FALSE(val.has_value());
    EXPECT_EQ(mock_transport->tx_count, 1);
}

TEST_F(MotorTest, StopCommand) {
    Result result = motor->stop();
    EXPECT_EQ(result, Result::SUCCESS);
    EXPECT_EQ(motor->get_status().state, MotorState::RESET);
    EXPECT_EQ((mock_transport->last_tx_frame.can_id & CAN_EFF_MASK), 0x04000001u);
}

TEST_F(MotorTest, SetZeroCommand) {
    Result result = motor->set_zero();
    EXPECT_EQ(result, Result::SUCCESS);
    EXPECT_EQ((mock_transport->last_tx_frame.can_id & CAN_EFF_MASK), 0x06000001u);
}

TEST_F(MotorTest, WriteParameterSuccess) {
    Result result = motor->write_parameter(ParamId::RUN_MODE, static_cast<uint8_t>(2));
    EXPECT_EQ(result, Result::SUCCESS);
}

TEST_F(MotorTest, SetupModeCurrent) {
    Result result = motor->setup_mode(RunMode::CURRENT);
    EXPECT_EQ(result, Result::SUCCESS);
    EXPECT_EQ(mock_transport->tx_count, 1); // Write(RUN_MODE)
}

TEST_F(MotorTest, OverrunDetection) {
    // The default overrun limit is 5.
    // 5 sends should succeed, and the 6th should fail.
    for (int i = 0; i < 5; i++) {
        Result res = motor->send_mit_control(0, 0, 0, 0, 0);
        EXPECT_EQ(res, Result::SUCCESS);
    }
    Result res = motor->send_mit_control(0, 0, 0, 0, 0);
    EXPECT_EQ(res, Result::ERR_OVERRUN);
}

TEST_F(MotorTest, ReceiveAllDispatch) {
    auto m2 = std::make_unique<Motor>(2, 0, Protocol::PRIVATE, PROFILE_RS00, mock_transport, 5);
    std::vector<Motor*> motors = { motor.get(), m2.get() };
    
    // push frames to mock transport manually
    struct can_frame f1{}; f1.can_id = (0x02800100) | CAN_EFF_FLAG; f1.can_dlc = 8; // feedback for ID 1
    struct can_frame f2{}; f2.can_id = (0x02800200) | CAN_EFF_FLAG; f2.can_dlc = 8; // feedback for ID 2
    
    {
        std::lock_guard<std::mutex> lock(mock_transport->mutex_);
        mock_transport->rx_queue.push_back(f1);
        mock_transport->rx_queue.push_back(f2);
    }
    
    std::thread t([&]() { Motor::receive_all(mock_transport, motors); });
    
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    mock_transport->disconnect();
    t.join();
    
    // Check if both motors parsed the frame (unknown_frame_count stays 0, angle is 0 etc but state updated)
    EXPECT_EQ(motor->get_status().motor_id, 1);
    EXPECT_EQ(m2->get_status().motor_id, 2); 
    // State should be RUN (2) because (2 << 22) is in the can_id
    EXPECT_EQ(motor->get_status().state, MotorState::RUN);
    EXPECT_EQ(m2->get_status().state, MotorState::RUN);
}

TEST_F(MotorTest, ConcurrentSendReceive) {
    // Stress test thread safety
    std::atomic<bool> running{true};
    
    // RX thread continuously pushes mock replies
    std::thread tx_simulator([&]() {
        while (running) {
            struct can_frame f1{}; f1.can_id = (0x02800100) | CAN_EFF_FLAG; f1.can_dlc = 8;
            {
                std::lock_guard<std::mutex> lock(mock_transport->mutex_);
                if (mock_transport->rx_queue.size() < 10) {
                    mock_transport->rx_queue.push_back(f1);
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    });
    
    for (int i = 0; i < 1000; i++) {
        motor->send_mit_control(0, 0, 0, 0, 0);
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    
    running = false;
    tx_simulator.join();
    
    // No segfaults or deadlocks means it passed
    EXPECT_TRUE(true);
}

TEST_F(MotorTest, SetupModeAllModes) {
    // CURRENT (Write * 1 = 1 tx) (RUN_MODE)
    mock_transport->tx_count = 0;
    EXPECT_EQ(motor->setup_mode(RunMode::CURRENT), Result::SUCCESS);
    EXPECT_EQ(mock_transport->tx_count, 1);

    // SPEED (Write * 3 = 3 tx) (RUN_MODE + LIMIT_CUR + ACC_RAD)
    mock_transport->tx_count = 0;
    EXPECT_EQ(motor->setup_mode(RunMode::SPEED, 1.0f, 2.0f), Result::SUCCESS);
    EXPECT_EQ(mock_transport->tx_count, 3);

    // POSITION_CSP (Write * 2 = 2 tx) (RUN_MODE + LIMIT_SPD)
    mock_transport->tx_count = 0;
    EXPECT_EQ(motor->setup_mode(RunMode::POSITION_CSP, 1.0f), Result::SUCCESS);
    EXPECT_EQ(mock_transport->tx_count, 2);

    // POSITION (Write * 4 = 4 tx) (RUN_MODE + MAX_VEL + ACC_SET + DEC_SET)
    mock_transport->tx_count = 0;
    EXPECT_EQ(motor->setup_mode(RunMode::POSITION, 1.0f, 2.0f, 4.0f), Result::SUCCESS);
    EXPECT_EQ(mock_transport->tx_count, 4);
}

TEST_F(MotorTest, CustomOverrunLimit) {
    // Create motor with overrun_limit = 10
    auto custom_motor = std::make_unique<Motor>(1, 0, Protocol::PRIVATE, PROFILE_RS00, mock_transport, 10);
    
    for (int i = 0; i < 10; i++) {
        Result res = custom_motor->send_mit_control(0, 0, 0, 0, 0);
        EXPECT_EQ(res, Result::SUCCESS);
    }
    Result res = custom_motor->send_mit_control(0, 0, 0, 0, 0);
    EXPECT_EQ(res, Result::ERR_OVERRUN);
}

TEST_F(MotorTest, ConnectDisconnectTransitions) {
    auto custom_motor = std::make_unique<Motor>(1, 0, Protocol::PRIVATE, PROFILE_RS00, mock_transport);
    
    // initially connected because mock_transport starts connected
    EXPECT_TRUE(mock_transport->is_connected());
    
    custom_motor->disconnect();
    EXPECT_FALSE(mock_transport->is_connected());
    EXPECT_FALSE(custom_motor->get_status().is_connected);
    
    // trying to send should fail with ERR_NOT_CONNECTED
    EXPECT_EQ(custom_motor->send_mit_control(0, 0, 0, 0, 0), Result::ERR_NOT_CONNECTED);
    EXPECT_EQ(custom_motor->enable(), Result::ERR_NOT_CONNECTED);
    
    custom_motor->connect();
    // mock transport doesn't implement connect() reconnecting in this test, but it returns SUCCESS
    // Actually we just check that the Result is SUCCESS
    EXPECT_EQ(custom_motor->connect(), Result::SUCCESS);
}
