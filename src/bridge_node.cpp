/**
 * @file bridge_node.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Reliable TCP bridge for ros data transfer in unstable network.
 * It will send/receive the specified ROS topics in ../config/ros_topics.yaml
 * It uses zmq socket(PUB/SUB mode), which reconnects others autonomously and
 * supports 1-N pub-sub connection even with TCP protocol.
 * 
 * Note: This program relies on ZMQPP (c++ wrapper around ZeroMQ).
 *  sudo apt install libzmqpp-dev
 * 
 * Core Idea: One event watcher receives configured ROS topics; sub_cb() sends.
 * 
 * @version 1.0
 * @date 2023-01-01
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#include "bridge_node.hpp"
#include "received_frame.hpp"
#include "peer_address.hpp"
#include <algorithm>
#include <chrono>
#include <csignal>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <random>
#include <sys/eventfd.h>
#include <xmlrpcpp/XmlRpcClient.h>
#include <xmlrpcpp/XmlRpcUtil.h>

volatile std::sig_atomic_t startup_stop = 0;

// Master readiness has no event source. Each attempt owns one asynchronous,
// one-second request, shared by startup and the bound-master watcher.
bool read_master_pid(int& pid)
{
  XmlRpc::XmlRpcClient client(ros::master::getHost().c_str(), ros::master::getPort(), "/");
  XmlRpc::XmlRpcValue request, response;
  request[0] = ros::this_node::getName();
  if (!client.executeNonBlock("getPid", request)) return false;
  bool done = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (!startup_stop && recv_running.load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < deadline && !done) {
    client._disp.work(0.05);
    done = client.executeCheckDone(response);
  }
  if (!done || response.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      response.size() != 3 || response[0].getType() != XmlRpc::XmlRpcValue::TypeInt ||
      static_cast<int>(response[0]) != 1 ||
      response[2].getType() != XmlRpc::XmlRpcValue::TypeInt) return false;
  pid = static_cast<int>(response[2]);
  return true;
}

// /* send messages frequency control */
// // this is the original freq control func that has been deprecated
// bool send_freq_control(int i)
// {
//   ros::Time t_now = ros::Time::now(); 
//   bool discard_flag;
//   if ((t_now - sub_t_last[i]).toSec() * sendTopics[i].max_freq < 1.0) {
//     discard_flag = true;
//   }
//   else {
//     discard_flag = false;
//     sub_t_last[i] = t_now; 
//   }
//   return discard_flag; // flag of discarding this message
// }

/* send messages frequency control */
bool send_freq_control(int i)
{
  bool discard_flag;
  ros::Time t_now = ros::Time::now(); 
  // check whether the send of this message will exceed the freq limit in the last period
  if ((send_num[i] + 1) / (t_now - sub_t_last[i]).toSec() > sendTopics[i].max_freq) {
    discard_flag = true;
  }
  else {
    discard_flag = false;
    send_num[i] ++;
  }
  // freq control period (1s)
  if ((t_now - sub_t_last[i]).toSec() > 1.0){
    sub_t_last[i] = t_now;
    send_num[i] = 0;
  }
  return discard_flag; // flag of discarding this message
}

/* uniform callback functions for ROS subscribers */
template <typename T, int i>
void sub_cb(const T &msg)
{
  /* frequency control */
  auto ignore_flag = send_freq_control(i);
  if (ignore_flag){
    return; // discard this message sending, abort
  }

  /* serialize the sending messages into send_buffer */
  namespace ser = ros::serialization;
  size_t data_len = ser::serializationLength(msg); // bytes length of msg
  std::unique_ptr<uint8_t[]> send_buffer(new uint8_t[data_len]);  // create a dynamic length array
  ser::OStream stream(send_buffer.get(), data_len);
  ser::serialize(stream, msg);

  /* zmq send message */
  zmqpp::message send_array;
  send_array << data_len; 
  /* equal to:
    send_array.add_raw(reinterpret_cast<void const*>(&data_len), sizeof(size_t));
  */
  send_array.add_raw(reinterpret_cast<void const *>(send_buffer.get()), data_len);
  senders[i]->send(send_array, true);
}


/* uniform deserialize and publish the receiving messages */
template<typename T>
void deserialize_pub(uint8_t* buffer_ptr, size_t msg_size, int i)
{
  // consumeReceivedFrame has already checked the ROS uint32_t length bound.
  const T msg = swarm_ros_bridge::deserializeExact<T>(
      buffer_ptr, static_cast<uint32_t>(msg_size));
  topic_pubs[i].publish(msg);
}


void recv_subscriber_change(int i, bool connected)
{
  if (connected)
    recv_subscribers[i]->fetch_add(1, std::memory_order_release);
  else
    recv_subscribers[i]->fetch_sub(1, std::memory_order_release);
  const uint64_t wake = 1;
  const ssize_t written = write(recv_wakeup, &wake, sizeof(wake));
  (void)written; // EAGAIN means a wakeup is already pending.
}

/* All socket operations and retry timers stay on this worker, off the ROS path. */
void recv_func()
{
  using Clock = std::chrono::steady_clock;
  struct Peer {
    std::unique_ptr<zmqpp::socket> socket, monitor;
    std::vector<int> topics;
    std::string url;
    bool active = false, connected = false, pending_log = false;
    int backoff_ms = 250;
    unsigned attempts = 0, suppressed = 0;
    Clock::time_point retry_at = Clock::now();
    Clock::time_point log_at = Clock::now();
  };
  try {
    std::mt19937 random(std::random_device{}());
    std::vector<Peer> peers;
    std::map<std::string, std::size_t> peer_indices;
    zmqpp::poller poller;
    poller.add(recv_wakeup);
    for (int i = 0; i < len_recv; ++i) {
      const std::string url = "tcp://" + recvTopics[i].ip + ":" + std::to_string(recvTopics[i].port);
      auto inserted = peer_indices.emplace(url, peers.size());
      if (!inserted.second) {
        peers[inserted.first->second].topics.push_back(i);
        continue;
      }
      peers.emplace_back();
      auto& peer = peers.back();
      peer.url = url;
      peer.topics.push_back(i);
      peer.socket.reset(new zmqpp::socket(context, zmqpp::socket_type::sub));
      peer.socket->set(zmqpp::socket_option::linger, 0);
      peer.socket->set(zmqpp::socket_option::receive_high_water_mark, 1);
      // Surface a silent link loss as a close event without application probes.
      peer.socket->set(zmqpp::socket_option::tcp_keepalive, 1);
      peer.socket->set(zmqpp::socket_option::tcp_keepalive_idle, 10);
      peer.socket->set(zmqpp::socket_option::tcp_keepalive_count, 3);
      peer.socket->set(zmqpp::socket_option::tcp_keepalive_interval, 10);
      // Native ZeroMQ retries use additive jitter. Schedule full jitter ourselves.
      peer.socket->set(zmqpp::socket_option::reconnect_interval, -1);
      const int connect_timeout_ms = 1000;
      if (zmq_setsockopt(*peer.socket, ZMQ_CONNECT_TIMEOUT,
                        &connect_timeout_ms, sizeof(connect_timeout_ms)) != 0 ||
          zmq_setsockopt(*peer.socket, ZMQ_HANDSHAKE_IVL,
                        &connect_timeout_ms, sizeof(connect_timeout_ms)) != 0)
        throw std::runtime_error("Cannot set ZeroMQ connection deadlines");
      peer.socket->subscribe("");
      const std::string monitor_url = "inproc://bridge-peer-" + std::to_string(i);
      peer.socket->monitor(monitor_url, ZMQ_EVENT_CONNECTED | ZMQ_EVENT_CLOSED |
                                       ZMQ_EVENT_DISCONNECTED);
      peer.monitor.reset(new zmqpp::socket(context, zmqpp::socket_type::pair));
      peer.monitor->set(zmqpp::socket_option::linger, 0);
      peer.monitor->connect(monitor_url);
      poller.add(*peer.socket);
      poller.add(*peer.monitor);
    }

    auto disconnect = [](Peer& peer) {
      // With native retries disabled, ZeroMQ may already have removed the
      // endpoint by the time its close event reaches this watcher.
      if (peer.active && zmq_disconnect(*peer.socket, peer.url.c_str()) != 0 &&
          zmq_errno() != ENOENT)
        throw std::runtime_error(zmq_strerror(zmq_errno()));
      peer.active = peer.connected = false;
    };
    auto failed = [&random, &disconnect](Peer& peer) {
      disconnect(peer);
      if (peer.pending_log) ++peer.suppressed;
      peer.pending_log = true;
      const int delay = std::uniform_int_distribution<int>(0, peer.backoff_ms)(random);
      peer.retry_at = Clock::now() + std::chrono::milliseconds(delay);
      peer.backoff_ms = std::min(peer.backoff_ms * 2, 30000);
    };

    while (recv_running.load(std::memory_order_acquire)) {
      long timeout_ms = -1;
      for (auto& peer : peers) {
        const bool wanted = std::any_of(peer.topics.begin(), peer.topics.end(), [](int i) {
          return recv_subscribers[i]->load(std::memory_order_acquire) != 0;
        });
        if (!wanted) {
          disconnect(peer);
          peer.pending_log = false;
          peer.backoff_ms = 250;
          peer.retry_at = Clock::now();
          continue;
        }
        if (!peer.active && Clock::now() >= peer.retry_at) {
          peer.active = true; // One asynchronous connect in flight per endpoint.
          ++peer.attempts;
          peer.socket->connect(peer.url);
        }
        auto deadline = [&timeout_ms](Clock::time_point at) {
          const long delay = std::max<long>(0, std::chrono::duration_cast<
              std::chrono::milliseconds>(at - Clock::now()).count() + 1);
          if (timeout_ms < 0 || delay < timeout_ms) timeout_ms = delay;
        };
        if (!peer.active) deadline(peer.retry_at);
        if (peer.pending_log) {
          if (Clock::now() >= peer.log_at) {
            ROS_INFO("[bridge_node] Peer %s status=%s attempts=%u suppressed=%u",
                     peer.url.c_str(), peer.connected ? "connected" : "unavailable",
                     peer.attempts, peer.suppressed);
            std::fflush(stdout);
            peer.pending_log = false;
            peer.suppressed = 0;
            peer.log_at = Clock::now() + std::chrono::seconds(10);
          } else {
            deadline(peer.log_at);
          }
        }
      }
      poller.poll(timeout_ms);
      if (poller.has_input(recv_wakeup)) {
        uint64_t wake;
        const ssize_t received = read(recv_wakeup, &wake, sizeof(wake));
        (void)received;
      }
      for (auto& peer : peers) {
        zmqpp::message event;
        while (peer.monitor->receive(event, true)) {
          uint16_t kind;
          std::memcpy(&kind, event.raw_data(0), sizeof(kind));
          if (kind == ZMQ_EVENT_CONNECTED && peer.active) {
            peer.connected = true;
            if (peer.pending_log) ++peer.suppressed;
            peer.pending_log = true;
          } else if (peer.active && (kind == ZMQ_EVENT_CLOSED ||
                                     kind == ZMQ_EVENT_DISCONNECTED)) {
            failed(peer);
          }
          event = zmqpp::message();
        }
        // Preserve the multipart wire format; CONFLATE discards its length part.
        // Drain a bounded batch and publish only the latest accepted sample.
        zmqpp::message frame, latest;
        for (int n = 0; n < 64 && peer.socket->receive(frame, true); ++n) {
          latest = std::move(frame);
          frame = zmqpp::message();
        }
        if (latest.parts() && peer.connected) {
          std::string rejection;
          if (swarm_ros_bridge::consumeReceivedFrame(
                  latest, [&peer](uint8_t* bytes, uint32_t length) {
                    for (int i : peer.topics)
                      if (recv_subscribers[i]->load(std::memory_order_acquire) != 0)
                        deserialize_publish(bytes, length, recvTopics[i].type, i);
                  }, rejection)) {
            // A TCP accept alone is not a successful protocol connection.
            peer.backoff_ms = 250;
          } else {
            ROS_WARN_THROTTLE(10.0, "[bridge_node] Rejected frame from %s: %s",
                              peer.url.c_str(), rejection.c_str());
          }
        }
      }
    }
  } catch (const std::exception& error) {
    bridge_failed.store(true, std::memory_order_release);
    ROS_FATAL("[bridge_node] Receive watcher failed: %s", error.what());
    ros::requestShutdown();
  }
}

/* Close the send socket and unsubscribe the ROS topic. */
void stop_send(int i)
{
  senders[i]->close();
  topic_subs[i].shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_bridge");
  // Native roscpp waits indefinitely, but retries every 50 ms. Wait for this
  // hard resource ourselves to avoid its retry and diagnostic storm.
  const auto previous_int = std::signal(SIGINT, [](int) { startup_stop = 1; });
  const auto previous_term = std::signal(SIGTERM, [](int) { startup_stop = 1; });
  struct StartupErrors : XmlRpc::XmlRpcErrorHandler {
    void error(const char*) override {} // Failures are summarized below.
  } startup_errors;
  auto* previous_errors = XmlRpc::XmlRpcErrorHandler::getErrorHandler();
  XmlRpc::XmlRpcErrorHandler::setErrorHandler(&startup_errors);
  std::mt19937 random(std::random_device{}());
  int bound_master_pid = 0, backoff_ms = 250;
  unsigned attempts = 0, suppressed = 0;
  auto log_at = std::chrono::steady_clock::now();
  while (!startup_stop) {
    ++attempts;
    if (read_master_pid(bound_master_pid)) break;
    if (std::chrono::steady_clock::now() >= log_at) {
      std::cout << "[bridge_node] ROS master unavailable attempts=" << attempts
                << " suppressed=" << suppressed << std::endl;
      suppressed = 0;
      log_at = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    } else {
      ++suppressed;
    }
    const int delay = std::uniform_int_distribution<int>(0, backoff_ms)(random);
    const auto retry_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(delay);
    backoff_ms = std::min(backoff_ms * 2, 30000);
    while (!startup_stop && std::chrono::steady_clock::now() < retry_at)
      std::this_thread::sleep_for(std::min(std::chrono::milliseconds(100),
          std::chrono::duration_cast<std::chrono::milliseconds>(
              retry_at - std::chrono::steady_clock::now()) + std::chrono::milliseconds(1)));
  }
  XmlRpc::XmlRpcErrorHandler::setErrorHandler(previous_errors);
  std::signal(SIGINT, previous_int);
  std::signal(SIGTERM, previous_term);
  if (startup_stop) return 0;
  std::cout << "[bridge_node] ROS master bound attempts=" << attempts
            << " suppressed=" << suppressed << std::endl;
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_public;
  ns = ros::this_node::getNamespace(); // namespace of this node
  std::string bound_run_id;
  const bool has_run_id = nh_public.getParamCached("/run_id", bound_run_id);

  std::cout << "--------[bridge_node]-------" << std::endl;
  std::cout << "namespaces=" << ns << std::endl;

  //************************ Parse configuration file **************************
  // get hostnames and IPs
  const bool has_peer_aliases = nh.getParam("IP", ip_xml);
  if (has_peer_aliases && ip_xml.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_ERROR("[bridge node] IP aliases must be an object");
    return 1;
  }
  // get "send topics" params (topic_name, topic_type, IP, port)
  if (nh.getParam("send_topics", send_topics_xml)){
    ROS_ASSERT(send_topics_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
    len_send = send_topics_xml.size();
  }
  else{
    ROS_WARN("[bridge node] No send_topics found in the configuration!");
    len_send = 0;
  }
  // get "receive topics" params (topic_name, topic_type, IP, port)
  if (nh.getParam("recv_topics", recv_topics_xml)){
    ROS_ASSERT(recv_topics_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
    len_recv = recv_topics_xml.size();
  }
  else{
    ROS_WARN("[bridge node] No recv_topics found in the configuration!");
    len_recv = 0;
  }

  if (len_send > SUB_MAX)
  {
    ROS_FATAL("[bridge_node] The number of send topics in configuration exceeds the limit %d!", SUB_MAX);
    return 2;
  }

  std::cout << "-------------IP------------" << std::endl;
  if (has_peer_aliases) for (auto iter = ip_xml.begin(); iter != ip_xml.end(); ++iter)
  {
    std::string host_name = iter->first;
    std::string host_ip = iter->second;
    std::cout << host_name << " : " << host_ip << std::endl;
    if (ip_map.find(host_name) != ip_map.end())
    { // ip_xml will never contain same names actually.
      ROS_WARN("[bridge node] IPs with the same name in configuration %s!", host_name.c_str());
    }
    ip_map[host_name] = host_ip;
  }

  std::cout << "--------send topics--------" << std::endl;
  std::set<int> srcPorts; // for duplicate check 
  for (int32_t i=0; i < len_send; ++i)
  {
    ROS_ASSERT(send_topics_xml[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue send_topic_xml = send_topics_xml[i];
    std::string topic_name = send_topic_xml["topic_name"];
    std::string msg_type = send_topic_xml["msg_type"];
    int max_freq = send_topic_xml["max_freq"];
    std::string srcIP;
    try { srcIP = swarm_ros_bridge::peerAddress(std::string(send_topic_xml["srcIP"]), ip_map); }
    catch (const std::exception& error) { ROS_ERROR("%s", error.what()); return 1; }
    int srcPort = send_topic_xml["srcPort"];
    TopicInfo topic = {.name=topic_name, .type=msg_type, .max_freq=max_freq, .ip=srcIP, .port=srcPort};
    sendTopics.emplace_back(topic);
    // check for duplicate ports:
    if (srcPorts.find(srcPort) != srcPorts.end()) {
      ROS_FATAL("[bridge_node] Send topics with the same srcPort %d in configuration!", srcPort);
      return 3;
    }
    srcPorts.insert(srcPort); // for duplicate check 
    if (topic.name.at(0) != '/') {
      std::cout << ns;
      if (ns != "/") {std::cout << "/";}
    }  // print namespace prefix if topic.name is not global
    std::cout << topic.name << "  " << topic.max_freq << "Hz(max)" << std::endl;
  }

  std::cout << "-------receive topics------" << std::endl;
  for (int32_t i=0; i < len_recv; ++i)
  {
    ROS_ASSERT(recv_topics_xml[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue recv_topic_xml = recv_topics_xml[i];
    std::string topic_name = recv_topic_xml["topic_name"];
    std::string msg_type = recv_topic_xml["msg_type"];
    int max_freq = recv_topic_xml["max_freq"];
    std::string srcIP;
    try { srcIP = swarm_ros_bridge::peerAddress(std::string(recv_topic_xml["srcIP"]), ip_map); }
    catch (const std::exception& error) { ROS_ERROR("%s", error.what()); return 1; }
    int srcPort = recv_topic_xml["srcPort"];
    TopicInfo topic = {.name=topic_name, .type=msg_type, .max_freq=max_freq, .ip=srcIP, .port=srcPort};
    recvTopics.emplace_back(topic);
    if (topic.name.at(0) != '/') {
      std::cout << ns;
      if (ns != "/") {std::cout << "/";}
    }  // print namespace prefix if topic name is not global
    std::cout << topic.name << "  (from " << recv_topic_xml["srcIP"]  << ")" << std::endl;
  }

  // ********************* zmq socket initialize ***************************
  // send sockets (zmq socket PUB mode)
  for (int32_t i=0; i < len_send; ++i)
  {
    const std::string url = "tcp://" + sendTopics[i].ip + ":" + std::to_string(sendTopics[i].port);
    std::unique_ptr<zmqpp::socket> sender(new zmqpp::socket(context, zmqpp::socket_type::pub));
    sender->set(zmqpp::socket_option::linger, 0);
    sender->set(zmqpp::socket_option::send_high_water_mark, 1);
    sender->set(zmqpp::socket_option::tcp_keepalive, 1);
    sender->set(zmqpp::socket_option::tcp_keepalive_idle, 10);
    sender->set(zmqpp::socket_option::tcp_keepalive_count, 3);
    sender->set(zmqpp::socket_option::tcp_keepalive_interval, 10);
    sender->bind(url);
    senders.emplace_back(std::move(sender)); //sender is now released by std::move
  }

  recv_wakeup = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
  if (recv_wakeup < 0) {
    ROS_FATAL("[bridge_node] Cannot create receive watcher wakeup");
    return 1;
  }
  for (int i = 0; i < len_recv; ++i)
    recv_subscribers.emplace_back(new std::atomic<unsigned>(0));

  // ******************* ROS subscribe and publish *************************
  //ROS topic subsrcibe and send
  for (int32_t i=0; i < len_send; ++i)
  {
    sub_t_last.emplace_back(ros::Time::now()); // freq control period start time
    send_num.emplace_back(0); // the send messages number in a period
    ros::Subscriber subscriber;
    // The uniform callback function is sub_cb()
    subscriber = topic_subscriber(sendTopics[i].name, sendTopics[i].type, nh_public, i);
    topic_subs.emplace_back(subscriber);
    // use topic_subs[i].shutdown() to unsubscribe
  }

  // ROS topic receive and publish
  for (int32_t i=0; i < len_recv; ++i) 
  {
    ros::Publisher publisher;
    publisher = topic_publisher(recvTopics[i].name, recvTopics[i].type, nh_public, i);
    topic_pubs.emplace_back(publisher);
  }

  std::thread recv_thread(&recv_func);
  // A master has no replacement event. Check its identity on one background
  // watcher every ten seconds, with a bounded, asynchronous XMLRPC request.
  std::mutex master_wait_mutex;
  std::condition_variable master_wait;
  std::thread master_thread([&] {
    while (recv_running.load(std::memory_order_acquire)) {
      int pid = 0;
      const bool contacted = read_master_pid(pid);
      if (!recv_running.load(std::memory_order_acquire)) return;
      std::string run_id;
      if (!contacted || pid != bound_master_pid ||
          (has_run_id && (!nh_public.getParamCached("/run_id", run_id) || run_id != bound_run_id))) {
        ROS_ERROR("[bridge_node] Bound ROS master lost or replaced; stopping");
        bridge_failed.store(true, std::memory_order_release);
        ros::requestShutdown();
        return;
      }
      std::unique_lock<std::mutex> lock(master_wait_mutex);
      master_wait.wait_for(lock, std::chrono::seconds(10), [] {
        return !recv_running.load(std::memory_order_acquire);
      });
    }
  });

  ros::spin();

  // ***************** stop send/receive ******************************
  for (int32_t i=0; i < len_send; ++i){
    stop_send(i);
  }

  {
    std::lock_guard<std::mutex> lock(master_wait_mutex);
    recv_running.store(false, std::memory_order_release);
  }
  master_wait.notify_one();
  const uint64_t wake = 1;
  const ssize_t written = write(recv_wakeup, &wake, sizeof(wake));
  (void)written;
  recv_thread.join();
  master_thread.join();

  for (int32_t i=0; i < len_recv; ++i){
    topic_pubs[i].shutdown(); // unadvertise after receive threads have stopped
  }
  close(recv_wakeup);
  
  return bridge_failed.load(std::memory_order_acquire) ? 1 : 0;
}
