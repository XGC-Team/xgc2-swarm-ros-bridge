#pragma once

#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <ros/serialization.h>
#include <zmqpp/message.hpp>

namespace swarm_ros_bridge {

// Keep the existing two-part wire format and native size_t metadata width.
// Validate the complete multipart envelope before allocating or copying payload.
template <typename Consumer>
bool consumeReceivedFrame(const zmqpp::message& frame, Consumer consume, std::string& error) {
  error.clear();
  try {
    if (frame.parts() != 2 || frame.size(0) != sizeof(std::size_t)) {
      error = "expected a native-size length field and one payload part";
      return false;
    }
    std::size_t data_len = 0;
    frame.get(data_len, 0);
    if (data_len != frame.size(1) ||
        data_len > std::numeric_limits<uint32_t>::max()) {
      error = "declared payload length does not match the ROS-sized payload";
      return false;
    }
    std::unique_ptr<uint8_t[]> buffer(new uint8_t[data_len]);
    if (data_len != 0) {
      std::memcpy(buffer.get(), frame.raw_data(1), data_len);
    }
    consume(buffer.get(), static_cast<uint32_t>(data_len));
    return true;
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
}

// Do not publish partial messages or silently accept trailing serialized bytes.
template <typename T>
T deserializeExact(uint8_t* buffer, uint32_t length) {
  T value;
  ros::serialization::IStream stream(buffer, length);
  ros::serialization::deserialize(stream, value);
  if (stream.getLength() != 0) {
    throw std::runtime_error("trailing bytes in serialized ROS message");
  }
  return value;
}

}  // namespace swarm_ros_bridge
