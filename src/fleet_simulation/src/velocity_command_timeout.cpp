#include <gz/msgs/twist.pb.h>

#include <chrono>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <memory>
#include <mutex>
#include <optional>
#include <sdf/Element.hh>
#include <stdexcept>
#include <string>

namespace fleet_simulation
{

class VelocityCommandTimeout final : public gz::sim::System,
                                     public gz::sim::ISystemConfigure,
                                     public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    for (const char * element : {"topic", "timeout"}) {
      if (!sdf->HasElement(element)) {
        throw std::invalid_argument(
          std::string("configure velocity command timeout failed: cause: <") + element +
          "> is missing");
      }
    }
    topic_ = sdf->Get<std::string>("topic");
    const double seconds = sdf->Get<double>("timeout");
    if (!(seconds > 0.0)) {
      throw std::invalid_argument(
        "configure velocity command timeout failed: cause: <timeout> must be positive, got " +
        std::to_string(seconds));
    }
    timeout_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(seconds));
    if (!node_.Subscribe(topic_, &VelocityCommandTimeout::OnCommand, this)) {
      throw std::runtime_error(
        "configure velocity command timeout failed: cause: subscribe to " + topic_ + " failed");
    }
    publisher_ = node_.Advertise<gz::msgs::Twist>(topic_);
    if (!publisher_) {
      throw std::runtime_error(
        "configure velocity command timeout failed: cause: advertise " + topic_ + " failed");
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    now_ = info.simTime;
    if (info.paused || stopped_ || !last_command_ || now_ - *last_command_ < timeout_) {
      return;
    }
    stopped_ = true;
    publisher_.Publish(gz::msgs::Twist());
  }

private:
  static bool IsZero(const gz::msgs::Twist & msg)
  {
    return msg.linear().x() == 0.0 && msg.linear().y() == 0.0 && msg.linear().z() == 0.0 &&
           msg.angular().x() == 0.0 && msg.angular().y() == 0.0 && msg.angular().z() == 0.0;
  }

  void OnCommand(const gz::msgs::Twist & msg)
  {
    if (IsZero(msg)) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    last_command_ = now_;
    stopped_ = false;
  }

  gz::transport::Node node_;
  gz::transport::Node::Publisher publisher_;
  std::string topic_;
  std::chrono::steady_clock::duration timeout_{};
  std::mutex mutex_;
  std::chrono::steady_clock::duration now_{};
  std::optional<std::chrono::steady_clock::duration> last_command_;
  bool stopped_{false};
};

}  // namespace fleet_simulation

GZ_ADD_PLUGIN(
  fleet_simulation::VelocityCommandTimeout, gz::sim::System,
  fleet_simulation::VelocityCommandTimeout::ISystemConfigure,
  fleet_simulation::VelocityCommandTimeout::ISystemPreUpdate)
