/*
 * OpenCNC - Communication Protocol Unit Tests
 */

#include <gtest/gtest.h>
#include "cnc_protocol.h"
#include "cnc_comm.h"

using namespace cnc::comm;

class CRC16Test : public ::testing::Test {};

TEST_F(CRC16Test, EmptyData) {
    uint8_t data[] = {};
    uint16_t crc = calculateCRC16(data, 0);
    EXPECT_EQ(crc, 0xFFFF);  // Initial value with no data
}

TEST_F(CRC16Test, SingleByte) {
    uint8_t data[] = {0x00};
    uint16_t crc = calculateCRC16(data, 1);
    EXPECT_NE(crc, 0);
}

TEST_F(CRC16Test, KnownValue) {
    // Test with known CRC-16-CCITT test vector
    // "123456789" should produce 0x29B1 for CRC-16-CCITT
    const char* test = "123456789";
    uint16_t crc = calculateCRC16(reinterpret_cast<const uint8_t*>(test), 9);
    EXPECT_EQ(crc, 0x29B1);
}

TEST_F(CRC16Test, DifferentDataProducesDifferentCRC) {
    uint8_t data1[] = {0x01, 0x02, 0x03};
    uint8_t data2[] = {0x01, 0x02, 0x04};
    
    uint16_t crc1 = calculateCRC16(data1, 3);
    uint16_t crc2 = calculateCRC16(data2, 3);
    
    EXPECT_NE(crc1, crc2);
}

class PacketEncodingTest : public ::testing::Test {};

TEST_F(PacketEncodingTest, MinimalPacket) {
    Packet packet;
    packet.command = CMD_GET_STATUS;
    packet.sequence = 0;
    packet.payloadLength = 0;
    
    std::vector<uint8_t> encoded = encodePacket(packet);
    
    // Minimum: START + CMD + SEQ + LEN + CRC(2) + END = 7 bytes
    EXPECT_GE(encoded.size(), 7);
    EXPECT_EQ(encoded.front(), PACKET_START);
    EXPECT_EQ(encoded.back(), PACKET_END);
}

TEST_F(PacketEncodingTest, PacketWithPayload) {
    Packet packet;
    packet.command = CMD_MOTION_SEGMENT;
    packet.sequence = 1;
    packet.payload = {0x01, 0x02, 0x03, 0x04};
    packet.payloadLength = 4;
    
    std::vector<uint8_t> encoded = encodePacket(packet);
    
    EXPECT_EQ(encoded.size(), 7 + 4);  // Header + payload
}

TEST_F(PacketEncodingTest, SequenceNumber) {
    Packet packet1;
    packet1.command = CMD_PING;
    packet1.sequence = 100;
    packet1.payloadLength = 0;
    
    std::vector<uint8_t> encoded = encodePacket(packet1);
    
    // Sequence should be at position 2
    EXPECT_EQ(encoded[2], 100);
}

class PacketDecodingTest : public ::testing::Test {};

TEST_F(PacketDecodingTest, ValidPacket) {
    // Create a valid packet
    Packet original;
    original.command = CMD_GET_STATUS;
    original.sequence = 5;
    original.payloadLength = 0;
    
    std::vector<uint8_t> encoded = encodePacket(original);
    
    Packet decoded;
    bool success = decodePacket(encoded.data(), encoded.size(), decoded);
    
    EXPECT_TRUE(success);
    EXPECT_EQ(decoded.command, original.command);
    EXPECT_EQ(decoded.sequence, original.sequence);
}

TEST_F(PacketDecodingTest, InvalidStartByte) {
    uint8_t data[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x55};
    
    Packet decoded;
    bool success = decodePacket(data, sizeof(data), decoded);
    
    EXPECT_FALSE(success);
}

TEST_F(PacketDecodingTest, InvalidEndByte) {
    uint8_t data[] = {0xAA, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00};
    
    Packet decoded;
    bool success = decodePacket(data, sizeof(data), decoded);
    
    EXPECT_FALSE(success);
}

TEST_F(PacketDecodingTest, InvalidCRC) {
    Packet original;
    original.command = CMD_PING;
    original.sequence = 0;
    original.payloadLength = 0;
    
    std::vector<uint8_t> encoded = encodePacket(original);
    
    // Corrupt the CRC
    encoded[encoded.size() - 3] ^= 0xFF;
    
    Packet decoded;
    bool success = decodePacket(encoded.data(), encoded.size(), decoded);
    
    EXPECT_FALSE(success);
}

TEST_F(PacketDecodingTest, PayloadExtraction) {
    Packet original;
    original.command = CMD_MOTION_SEGMENT;
    original.sequence = 0;
    original.payload = {0xDE, 0xAD, 0xBE, 0xEF};
    original.payloadLength = 4;
    
    std::vector<uint8_t> encoded = encodePacket(original);
    
    Packet decoded;
    bool success = decodePacket(encoded.data(), encoded.size(), decoded);
    
    EXPECT_TRUE(success);
    EXPECT_EQ(decoded.payloadLength, 4);
    EXPECT_EQ(decoded.payload.size(), 4);
    EXPECT_EQ(decoded.payload[0], 0xDE);
    EXPECT_EQ(decoded.payload[3], 0xEF);
}

class PositionEncodingTest : public ::testing::Test {};

TEST_F(PositionEncodingTest, EncodeZero) {
    double mm = 0.0;
    int32_t encoded = encodePosition(mm);
    EXPECT_EQ(encoded, 0);
}

TEST_F(PositionEncodingTest, EncodePositive) {
    double mm = 100.0;
    int32_t encoded = encodePosition(mm);
    EXPECT_EQ(encoded, 100000000);  // 100mm * 1000000 nm/mm
}

TEST_F(PositionEncodingTest, EncodeNegative) {
    double mm = -50.5;
    int32_t encoded = encodePosition(mm);
    EXPECT_EQ(encoded, -50500000);
}

TEST_F(PositionEncodingTest, RoundTrip) {
    double original = 123.456789;
    int32_t encoded = encodePosition(original);
    double decoded = decodePosition(encoded);
    
    // Should be accurate to nanometer
    EXPECT_NEAR(decoded, original, 0.000001);
}

class MotionSegmentPacketTest : public ::testing::Test {};

TEST_F(MotionSegmentPacketTest, Serialize) {
    MotionSegmentPacket seg;
    seg.segmentId = 12345;
    seg.flags = MOTION_FLAG_RAPID;
    seg.targetX = encodePosition(100.0);
    seg.targetY = encodePosition(50.0);
    seg.targetZ = encodePosition(25.0);
    seg.entryVelocity = 1000;
    seg.cruiseVelocity = 5000;
    seg.exitVelocity = 1000;
    seg.acceleration = 500;
    
    std::vector<uint8_t> data = seg.serialize();
    
    EXPECT_EQ(data.size(), sizeof(MotionSegmentPacket));
}

TEST_F(MotionSegmentPacketTest, Deserialize) {
    MotionSegmentPacket original;
    original.segmentId = 99999;
    original.flags = MOTION_FLAG_ARC_CW;
    original.targetX = encodePosition(200.0);
    original.targetY = encodePosition(-100.0);
    original.targetZ = encodePosition(0.0);
    
    std::vector<uint8_t> data = original.serialize();
    
    MotionSegmentPacket decoded;
    decoded.deserialize(data.data());
    
    EXPECT_EQ(decoded.segmentId, original.segmentId);
    EXPECT_EQ(decoded.flags, original.flags);
    EXPECT_EQ(decoded.targetX, original.targetX);
    EXPECT_EQ(decoded.targetY, original.targetY);
    EXPECT_EQ(decoded.targetZ, original.targetZ);
}

class RealtimeStatusTest : public ::testing::Test {};

TEST_F(RealtimeStatusTest, ParseStatus) {
    RealtimeStatusPacket status;
    status.state = STATE_RUNNING;
    status.alarmCode = ALARM_NONE;
    status.positionX = encodePosition(50.0);
    status.positionY = encodePosition(75.0);
    status.positionZ = encodePosition(10.0);
    status.feedrate = 1500;
    status.spindleRpm = 12000;
    status.queueDepth = 16;
    status.statusFlags = STATUS_FLAG_SPINDLE_ON | STATUS_FLAG_SPINDLE_CW;
    
    std::vector<uint8_t> data = status.serialize();
    
    RealtimeStatusPacket decoded;
    decoded.deserialize(data.data());
    
    EXPECT_EQ(decoded.state, STATE_RUNNING);
    EXPECT_EQ(decoded.positionX, status.positionX);
    EXPECT_EQ(decoded.feedrate, 1500);
    EXPECT_EQ(decoded.spindleRpm, 12000);
    EXPECT_TRUE(decoded.statusFlags & STATUS_FLAG_SPINDLE_ON);
}
