#include <gtest/gtest.h>
#include "robstride/protocols/protocol_mit.h"
#include "robstride/protocols/protocol_private.h"

using namespace robstride;
using namespace robstride::protocols;

TEST(ProtocolMITTest, EncodeDecodeControlRoundTrip) {
    ProtocolMIT proto;
    MotorProfile profile = PROFILE_RS00; // rs00 limits
    
    float pos = 1.0f;
    float vel = 2.0f;
    float kp = 3.0f;
    float kd = 0.5f;
    float torque = 0.1f;
    
    struct can_frame frame = proto.encode_mit_control(1, 0, torque, pos, vel, kp, kd, profile);
    
    // In MIT mode, motor_id is in CAN ID
    EXPECT_EQ(frame.can_id, 1);
    EXPECT_EQ(frame.can_dlc, 8);
    EXPECT_FALSE(frame.can_id & CAN_EFF_FLAG); // 11-bit standard ID
}

TEST(ProtocolMITTest, ResponseTypeDetection) {
    ProtocolMIT proto;
    
    struct can_frame frame{};
    
    // Mode 0 = FEEDBACK
    frame.can_id = (0 << 8) | 1;
    EXPECT_EQ(proto.get_response_type(frame), FrameType::FEEDBACK);
    
    // Mode 3 = READ_PARAM
    frame.can_id = (3 << 8) | 1;
    EXPECT_EQ(proto.get_response_type(frame), FrameType::READ_PARAM);
    
    // Mode 4 = WRITE_PARAM (same logical response type)
    frame.can_id = (4 << 8) | 1;
    EXPECT_EQ(proto.get_response_type(frame), FrameType::READ_PARAM);
}

TEST(ProtocolMITTest, ParseFeedback) {
    ProtocolMIT proto;
    MotorProfile profile = PROFILE_RS00;
    
    struct can_frame frame{};
    frame.can_id = (0 << 8) | 0; // Host ID is usually CAN ID for responses
    frame.can_dlc = 8;
    frame.data[0] = 5; // motor ID
    
    // Encode some mock data
    uint16_t pos_raw = rs_mit_math_encode_angle(1.5f);
    uint16_t vel_raw = rs_mit_math_encode_velocity(2.0f);
    uint16_t torq_raw = rs_mit_math_encode_torque(0.5f, profile.torque_limit);
    
    frame.data[1] = pos_raw >> 8;
    frame.data[2] = pos_raw & 0xFF;
    frame.data[3] = vel_raw >> 4;
    frame.data[4] = ((vel_raw & 0x0F) << 4) | (torq_raw >> 8);
    frame.data[5] = torq_raw & 0xFF;
    frame.data[6] = 0; // Temp upper
    frame.data[7] = 250; // Temp lower = 25.0 C
    
    MotorStatus status{};
    status.motor_id = 5; // Expected motor_id
    
    EXPECT_TRUE(proto.parse_feedback(frame, status, profile));
    EXPECT_NEAR(status.angle, 1.5f, 0.1f);
    EXPECT_NEAR(status.velocity, 2.0f, 0.1f);
    EXPECT_NEAR(status.torque, 0.5f, 0.1f);
    EXPECT_NEAR(status.temperature, 25.0f, 0.1f);
    EXPECT_EQ(status.state, MotorState::RUN);
}

TEST(ProtocolPrivateTest, ResponseTypeDetection) {
    ProtocolPrivate proto;
    
    struct can_frame frame{};
    
    // Type 2 = FEEDBACK
    frame.can_id = (2 << 24) | CAN_EFF_FLAG;
    EXPECT_EQ(proto.get_response_type(frame), FrameType::FEEDBACK);
    
    // Type 17 = READ_PARAM
    frame.can_id = (17 << 24) | CAN_EFF_FLAG;
    EXPECT_EQ(proto.get_response_type(frame), FrameType::READ_PARAM);
    
    // Type 0 = INFO
    frame.can_id = (0 << 24) | CAN_EFF_FLAG;
    EXPECT_EQ(proto.get_response_type(frame), FrameType::INFO);
}

TEST(ProtocolPrivateTest, ParseFeedback) {
    ProtocolPrivate proto;
    MotorProfile profile = PROFILE_RS00;
    
    struct can_frame frame{};
    // FEEDBACK type=2, RUN state=2, Motor ID=5
    frame.can_id = (2 << 24) | (2 << 22) | (5 << 8) | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    
    MotorStatus status{};
    EXPECT_TRUE(proto.parse_feedback(frame, status, profile));
    EXPECT_EQ(status.motor_id, 5);
    EXPECT_EQ(status.state, MotorState::RUN);
}
