#include "received_frame.hpp"
#include "peer_address.hpp"

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <rosgraph_msgs/Clock.h>

#include <iostream>
#include <vector>

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

zmqpp::message frame(std::size_t declared, const std::vector<uint8_t>& payload) {
  zmqpp::message result;
  result << declared;
  const uint8_t empty = 0;
  result.add_raw(payload.empty() ? &empty : payload.data(), payload.size());
  return result;
}

template <typename T>
std::vector<uint8_t> serialize(const T& message) {
  std::vector<uint8_t> bytes(ros::serialization::serializationLength(message));
  ros::serialization::OStream stream(bytes.data(), static_cast<uint32_t>(bytes.size()));
  ros::serialization::serialize(stream, message);
  return bytes;
}

void reject(zmqpp::message& message, const char* name) {
  bool published = false;
  std::string error;
  const bool accepted = swarm_ros_bridge::consumeReceivedFrame(
      message, [&published](uint8_t* bytes, uint32_t length) {
        (void)swarm_ros_bridge::deserializeExact<std_msgs::String>(bytes, length);
        published = true;
      }, error);
  require(!accepted && !published && !error.empty(), name);
  std::cout << "PASS " << name << '\n';
}

template <typename T>
void roundTrip(const T& input, const char* name) {
  const auto payload = serialize(input);
  auto message = frame(payload.size(), payload);
  std::string error;
  unsigned published = 0;
  require(swarm_ros_bridge::consumeReceivedFrame(
      message, [&payload, &published](uint8_t* bytes, uint32_t length) {
        const auto output = swarm_ros_bridge::deserializeExact<T>(bytes, length);
        require(serialize(output) == payload, "roundtrip bytes changed");
        ++published;
      }, error), name);
  require(published == 1 && error.empty(), "valid message did not publish exactly once");
  std::cout << "PASS " << name << '\n';
}

}  // namespace

int main() {
  try {
    const std::map<std::string,std::string> peers{{"robot","192.0.2.1"}};
    require(swarm_ros_bridge::peerAddress("robot",peers)=="192.0.2.1","named endpoint changed");
    require(swarm_ros_bridge::peerAddress("192.0.2.2",peers)=="192.0.2.2","literal endpoint rejected");
    bool invalid=false;try{swarm_ros_bridge::peerAddress("missing",peers);}catch(const std::invalid_argument&){invalid=true;}
    require(invalid,"unknown peer silently accepted");
    geometry_msgs::PoseStamped pose;pose.header.stamp=ros::Time(42,123);pose.header.frame_id="world";pose.pose.position.x=1.25;pose.pose.orientation.w=1;
    roundTrip(pose,"timestamped pose");
    geometry_msgs::TwistStamped measuredVelocity;measuredVelocity.header=pose.header;measuredVelocity.twist.linear.x=0.5;roundTrip(measuredVelocity,"timestamped velocity");
    nav_msgs::Path prediction;prediction.header=pose.header;prediction.poses.push_back(pose);roundTrip(prediction,"algorithm prediction");
    mavros_msgs::State state;state.armed=true;state.mode="OFFBOARD";roundTrip(state,"flight state");
    mavros_msgs::ExtendedState extended;extended.landed_state=2;roundTrip(extended,"landed state");
    rosgraph_msgs::Clock clock;clock.clock=ros::Time(42,123);roundTrip(clock,"simulation clock");
    zmqpp::message none;
    reject(none, "no parts");
    zmqpp::message missing;
    missing << std::size_t(4);
    reject(missing, "missing payload");
    auto extra = frame(4, {0, 0, 0, 0});
    extra << std::size_t(0);
    reject(extra, "extra part");
    zmqpp::message narrow;
    narrow << uint8_t(4);
    const uint32_t zero = 0;
    narrow.add_raw(&zero, sizeof(zero));
    reject(narrow, "wrong metadata width");
    auto oversized = frame(std::numeric_limits<std::size_t>::max(), {0});
    reject(oversized, "oversized declaration");
    auto shorter = frame(3, {0, 0, 0, 0});
    reject(shorter, "declaration shorter than payload");
    auto longer = frame(5, {0, 0, 0, 0});
    reject(longer, "declaration longer than payload");
    auto empty = frame(0, {});
    reject(empty, "empty ROS string payload");
    auto truncated = frame(4, {100, 0, 0, 0});
    reject(truncated, "truncated ROS string");
    auto trailing = frame(5, {0, 0, 0, 0, 99});
    reject(trailing, "trailing ROS bytes");

    // A bad message must not escape as an exception or prevent the next dispatch.
    std_msgs::String text;
    text.data = "valid-after-malformed";
    roundTrip(text, "String after rejected frames");
    text.data.clear();
    roundTrip(text, "empty String");
    std_msgs::Float32 scalar;
    scalar.data = 1.25f;
    roundTrip(scalar, "Float32");
    std_msgs::UInt32 counter;
    counter.data = 0xfedcba98u;
    roundTrip(counter, "UInt32");
    geometry_msgs::Twist twist;
    twist.linear.x = 0.25;
    twist.angular.z = -0.5;
    roundTrip(twist, "Twist");
    sensor_msgs::Imu imu;
    imu.orientation.w = 1.0;
    imu.linear_acceleration.z = 9.8066;
    roundTrip(imu, "Imu");
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
