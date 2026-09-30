#include "terrascout_hardware/can_bus.hpp"

#include <fcntl.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace terrascout_hardware
{

CanBus::~CanBus() { close(); }

CanBus::CanBus(CanBus && other) noexcept : socket_fd_(other.socket_fd_) { other.socket_fd_ = -1; }

CanBus & CanBus::operator=(CanBus && other) noexcept
{
  if (this != &other) {
    close();
    socket_fd_ = other.socket_fd_;
    other.socket_fd_ = -1;
  }
  return *this;
}

bool CanBus::open(const std::string & interface, const std::vector<struct can_filter> & filters)
{
  close();
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    last_error_ = std::string("create socket: ") + std::strerror(errno);
    return false;
  }
  auto fail = [this](const char * operation) {
    last_error_ = std::string(operation) + ": " + std::strerror(errno);
    close();
    return false;
  };

  if (
    setsockopt(
      socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
      static_cast<socklen_t>(filters.size() * sizeof(struct can_filter))) < 0) {
    return fail("set receive filter");
  }

  if (interface.size() >= IFNAMSIZ) {
    errno = ENAMETOOLONG;
    return fail("look up interface index");
  }
  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) return fail("look up interface index");

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    return fail("bind socket");
  }

  const int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0) return fail("read socket flags");
  if (fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) return fail("set socket non-blocking");
  return true;
}

void CanBus::close()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool CanBus::send(const struct can_frame & frame)
{
  if (socket_fd_ < 0) {
    last_error_ = "CAN socket is not open";
    return false;
  }
  ssize_t n = write(socket_fd_, &frame, sizeof(frame));
  if (n == static_cast<ssize_t>(sizeof(frame))) return true;
  last_error_ = n < 0 ? std::strerror(errno) : "incomplete CAN frame write";
  return false;
}

CanBus::RecvResult CanBus::recv(struct can_frame & frame)
{
  if (socket_fd_ < 0) {
    last_error_ = "CAN socket is not open";
    return RecvResult::Error;
  }
  ssize_t n = read(socket_fd_, &frame, sizeof(frame));
  if (n == static_cast<ssize_t>(sizeof(frame))) return RecvResult::Frame;
  if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return RecvResult::Empty;
  last_error_ = n < 0 ? std::strerror(errno) : "incomplete CAN frame read";
  return RecvResult::Error;
}

}  // namespace terrascout_hardware
