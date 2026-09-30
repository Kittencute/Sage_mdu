#include <gtest/gtest.h>

#include <cstring>

#include "terrascout_hardware/cubemars_protocol.hpp"

using namespace terrascout_hardware;

TEST(CubeMarsProtocol, EncodeRpmFrame)
{
  auto frame = cubemars::encode_rpm(0x01, 1000);
  EXPECT_EQ(frame.can_id, (cubemars::CMD_SET_RPM << 8) | 0x01 | CAN_EFF_FLAG);
  EXPECT_EQ(frame.can_dlc, 4);
  // 1000 = 0x000003E8 big-endian
  EXPECT_EQ(frame.data[0], 0x00);
  EXPECT_EQ(frame.data[1], 0x00);
  EXPECT_EQ(frame.data[2], 0x03);
  EXPECT_EQ(frame.data[3], 0xE8);
}

TEST(CubeMarsProtocol, EncodeNegativeRpm)
{
  auto frame = cubemars::encode_rpm(0x02, -500);
  int32_t decoded;
  std::memcpy(&decoded, frame.data, 4);
  decoded = __builtin_bswap32(static_cast<uint32_t>(decoded));
  EXPECT_EQ(decoded, -500);
}

TEST(CubeMarsProtocol, DecodeValidTelemetry)
{
  uint8_t data[8] = {};
  uint16_t pos = 1800;  // 180.0 deg
  int16_t speed = 500;
  uint16_t cur = 500;  // 5.0A
  data[0] = pos >> 8;
  data[1] = pos & 0xFF;
  data[2] = static_cast<uint8_t>(speed >> 8);
  data[3] = speed & 0xFF;
  data[4] = cur >> 8;
  data[5] = cur & 0xFF;
  data[6] = 35;
  data[7] = 0;

  auto tel = cubemars::decode_telemetry(data, 8, 9);
  ASSERT_TRUE(tel.has_value());
  EXPECT_NEAR(tel->position_deg, 180.0, 0.1);
  EXPECT_EQ(tel->motor_erpm, 5000);
  EXPECT_NEAR(tel->current_a, 5.0, 0.01);
  EXPECT_EQ(tel->temperature_c, 35);
  EXPECT_EQ(tel->error_code, 0);
}

TEST(CubeMarsProtocol, DecodeShortFrameReturnsNullopt)
{
  uint8_t data[3] = {0, 0, 0};
  auto tel = cubemars::decode_telemetry(data, 3, 9);
  EXPECT_FALSE(tel.has_value());
}
