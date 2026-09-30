#pragma once

#include <linux/can.h>

#include <string>
#include <vector>

namespace terrascout_hardware
{

class CanBus
{
public:
  CanBus() = default;
  ~CanBus();

  CanBus(const CanBus &) = delete;
  CanBus & operator=(const CanBus &) = delete;
  CanBus(CanBus && other) noexcept;
  CanBus & operator=(CanBus && other) noexcept;

  enum class RecvResult
  {
    Frame,
    Empty,
    Error
  };

  bool open(const std::string & interface, const std::vector<struct can_filter> & filters);
  void close();
  bool send(const struct can_frame & frame);
  RecvResult recv(struct can_frame & frame);
  bool is_open() const { return socket_fd_ >= 0; }
  const std::string & last_error() const { return last_error_; }

private:
  int socket_fd_ = -1;
  std::string last_error_;
};

}  // namespace terrascout_hardware
