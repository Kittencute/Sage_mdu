#include "terrascout_hardware/cubemars_protocol.hpp"

#include <arpa/inet.h>

namespace terrascout_hardware::cubemars
{

constexpr float DEG_PER_POSITION_UNIT = 0.1f;
constexpr int32_t ERPM_PER_SPEED_UNIT = 10;
constexpr float AMPERE_PER_CURRENT_UNIT = 0.01f;

static struct can_frame make_frame(uint32_t command, uint8_t motor_id, int32_t value)
{
  struct can_frame frame{};
  frame.can_id = (command << 8) | motor_id | CAN_EFF_FLAG;
  frame.can_dlc = 4;
  uint32_t be = htonl(static_cast<uint32_t>(value));
  std::memcpy(frame.data, &be, 4);
  return frame;
}

static int16_t read_int16(const uint8_t * data)
{
  return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

struct can_frame encode_rpm(uint8_t motor_id, int32_t rpm)
{
  return make_frame(CMD_SET_RPM, motor_id, rpm);
}

canid_t telemetry_can_id(uint8_t motor_id) { return (CMD_STATUS << 8) | motor_id | CAN_EFF_FLAG; }

std::optional<Telemetry> decode_telemetry(const uint8_t * data, size_t dlc, int /*gear_ratio*/)
{
  if (dlc != TELEMETRY_DLC) return std::nullopt;
  Telemetry t;
  t.position_deg = read_int16(&data[0]) * DEG_PER_POSITION_UNIT;
  t.motor_erpm = read_int16(&data[2]) * ERPM_PER_SPEED_UNIT;
  t.current_a = read_int16(&data[4]) * AMPERE_PER_CURRENT_UNIT;
  t.temperature_c = static_cast<int8_t>(data[6]);
  t.error_code = data[7];
  return t;
}

const char * describe_error(uint8_t error_code)
{
  switch (error_code) {
    case 0:
      return "no fault";
    case 1:
      return "motor over-temperature fault";
    case 2:
      return "over-current fault";
    case 3:
      return "over-voltage fault";
    case 4:
      return "under-voltage fault";
    case 5:
      return "encoder fault";
    case 6:
      return "MOSFET over-temperature fault";
    case 7:
      return "motor stall";
    default:
      return "code not listed in the servo-mode upload message protocol";
  }
}

}  // namespace terrascout_hardware::cubemars
