#include <mpc-rbt-solution/Sender.hpp>
#include <nlohmann/json.hpp>
#include <cstring>

void Sender::Node::run()
{
  while (errno != EINTR) {
    if ((std::chrono::steady_clock::now() - timer_tick) < timer_period) continue;
    timer_tick = std::chrono::steady_clock::now();

    callback();
  }
}

void Sender::Node::onDataTimerTick()
{
  data.x += 1.0;
  data.y += 1.0;
  data.z += 1.0;

  data.timestamp = static_cast<uint64_t>(
    std::chrono::system_clock::now().time_since_epoch().count());

  nlohmann::json j;
  Utils::Message::to_json(j, data);
  const std::string s = j.dump();
  data.frame = j.dump();


  Socket::IPFrame frame{};
  frame.address = config.remoteAddress;
  frame.port = config.remotePort;

  const size_t n = std::min(frame.serializedData.size(), s.size());
  std::memcpy(frame.serializedData.data(), s.data(), n);
  frame.dataSize = static_cast<ssize_t>(n);

  send(frame);

  RCLCPP_INFO(logger, "Sending to %s:%u | stamp=%lu | x=%.2f y=%.2f z=%.2f | bytes=%ld",
              frame.address.c_str(), frame.port,
              data.timestamp, data.x, data.y, data.z,
              static_cast<long>(frame.dataSize));
}
