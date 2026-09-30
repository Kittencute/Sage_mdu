#pragma once

#include <linux/can.h>

#include <cstdint>
#include <cstring>
#include <optional>

namespace terrascout_hardware::cubemars
{

constexpr uint32_t CMD_SET_DUTY = 0;
constexpr uint32_t CMD_SET_CURRENT = 1;
constexpr uint32_t CMD_SET_BRAKE = 2;
constexpr uint32_t CMD_SET_RPM = 3;
constexpr uint32_t CMD_SET_POS = 4;
constexpr uint32_t CMD_SET_ZERO = 5;
constexpr uint32_t CMD_SET_POS_SPD = 6;
constexpr uint32_t CMD_STATUS = 0x29;

constexpr size_t TELEMETRY_DLC = 8;
constexpr canid_t TELEMETRY_ID_MASK = CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK;

struct Telemetry
{
  float position_deg;
  int32_t motor_erpm;
  float current_a;
  int8_t temperature_c;
  uint8_t error_code;
};

struct can_frame encode_rpm(uint8_t motor_id, int32_t rpm);

canid_t telemetry_can_id(uint8_t motor_id);

std::optional<Telemetry> decode_telemetry(const uint8_t * data, size_t dlc, int gear_ratio);

const char * describe_error(uint8_t error_code);

}  // namespace terrascout_hardware::cubemars
