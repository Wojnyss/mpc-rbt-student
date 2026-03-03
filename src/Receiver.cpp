#include <mpc-rbt-solution/Receiver.hpp>
#include <nlohmann/json.hpp>

void Receiver::Node::run()
{
  while (errno != EINTR) {
    RCLCPP_INFO(logger, "Waiting for data ...");
    Socket::IPFrame frame{};
    if (receive(frame)) {
      RCLCPP_INFO(logger, "Received data from host: '%s:%d'", frame.address.c_str(), frame.port);

      callback(frame);

    } else {
      RCLCPP_WARN(logger, "Failed to receive data.");
    }
  }
}

void Receiver::Node::onDataReceived(const Socket::IPFrame & frame)
{
  const std::string s(
    reinterpret_cast<const char*>(frame.serializedData.data()),
    static_cast<size_t>(frame.dataSize)
  );
 
  data.frame = s;
  const auto j = nlohmann::json::parse(s);

  // Call the Message method (NOT Utils::from_json)
  Utils::Message::from_json(j, data);

  RCLCPP_INFO(logger, "Received from %s:%u | stamp=%lu | x=%.2f y=%.2f z=%.2f | bytes=%ld",
              frame.address.c_str(), frame.port,
              data.timestamp, data.x, data.y, data.z,
              static_cast<long>(frame.dataSize));
}
