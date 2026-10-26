#pragma once
#include <arpa/inet.h>
#include <map>
#include <stdexcept>
#include <string>

namespace swarm_ros_bridge {
// Existing named peers remain supported. A frozen literal address needs no
// second alias map, and an unknown name must not become an empty endpoint.
inline std::string peerAddress(const std::string& name,
                               const std::map<std::string, std::string>& peers) {
  const auto found = peers.find(name);
  if (found != peers.end() && !found->second.empty()) return found->second;
  in_addr address{};
  if (inet_pton(AF_INET, name.c_str(), &address) == 1) return name;
  throw std::invalid_argument("Unknown bridge peer: " + name);
}
}
