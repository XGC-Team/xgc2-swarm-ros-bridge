# swarm_ros_bridge

## XGC2 Product

**Legacy compatibility.** This repository packages the upstream ZeroMQ `bridge_node` for ROS Melodic
and Noetic so that the existing XGC2 robot products keep the ROS bridging they use today. The launch files, configuration format and multipart wire
format stay compatible. Runtime fixes keep the existing ZeroMQ path working.

- **Not part of same-entity aggregation.** The package takes no part in aggregation: it is not an aggregated module,
  needs no aggregation host or adapter integration, and has no aggregation work items. Aggregation work on the
  platform does not require migrating, rewriting or removing it.
- **Not the place for new communication work.** More reliable or more efficient communication is to be developed
  separately, on Zenoh together with aggregation. That work does not happen in this package: there is no second
  protocol and no Zenoh stack here, no new communication feature is planned, and existing consumers are not required
  to move. Only fixes that keep the existing ZeroMQ path working belong here.

```bash
sudo apt install ros-melodic-swarm-ros-bridge
sudo apt install ros-noetic-swarm-ros-bridge
```

Robot-specific peer/topic YAML belongs in the consuming robot product. The
compiled message set is:

- `sensor_msgs/Imu`
- `geometry_msgs/Twist`
- `std_msgs/String`
- `std_msgs/Float32`
- `std_msgs/UInt32`

CI builds amd64 and arm64 debs for both distributions inside XGC2 `xgc2-build`
images. Extra ROS message types still need a rebuild because headers are
registered at compile time.

## Parallel startup and reconnect

Start the node in parallel with its ROS master, publishers and remote bridges.
Use `roslaunch --wait` when launching against a session master so roslaunch waits
for that master instead of starting another one. Configuration still comes from
private ROS parameters. The node waits for its hard master resource indefinitely
with full jitter: 250 ms initially, doubling to a 30 s cap, one bounded XMLRPC
request at a time. SIGINT and SIGTERM cancel this wait. Once bound, one background
watcher checks the master's PID every 10 s with a 1 s deadline and reads cached
`/run_id` updates. Master loss or replacement stops the node; data peer loss does
not. This legacy ROS node reports status through logs, without an XRPC service.

Configured topics register through normal ROS subscriptions and advertisements;
no topic discovery polling or startup topic wait is needed. Publisher updates
handle topics appearing later and publishers restarting. A receive connection
starts with the first subscriber to its ROS output and stops after the last one
leaves. Multiple outputs from one endpoint share a socket and retry schedule.
One event watcher handles all ZeroMQ peers, using close events and exponential
full jitter from 250 ms to 30 s. It resets backoff after a valid sample, keeps one
attempt in flight per endpoint, and logs at most once per endpoint per 10 s with
attempt and suppression counts. TCP keepalives detect silent link loss with
a 10 s idle interval and three probes 10 s apart.

ROS queues and ZeroMQ high water marks are one sample. Offline PUB data is dropped;
receive batches publish their latest sample rather than replaying a backlog.
The multipart length/payload envelope is retained because ZeroMQ conflation
cannot preserve multipart frames. The bridge owns no mutable remote desired
state; ZeroMQ replays the static subscription once on each new connection.

The tests require `rospy`, `roslaunch` and `python3-zmq`, and always use private
master ports from 20000 through 40000 and a temporary `ROS_HOME`. After configuring
with `CATKIN_ENABLE_TESTING=ON` and building the test targets with `-j2`, run CTest
from the build directory:

```bash
ctest -LE long --output-on-failure  # Fast startup, publisher restart and shutdown checks
ctest -L long -V                   # Two measured 60-second outages and recovery
```

The Noetic source-contract CI script runs all tests, including the `long` label.
No hardware or Gazebo is needed, and each test stops and waits for its children.

The note below is the upstream author's advice and does not describe XGC2 plans.

> :heavy_exclamation_mark: (Updated on 2026.3.20) The author strongly recommend you to move to the zenoh ROS bridge, which has automatic ROS topic discovery mechanism and better port management. ROS1 zenoh bridge: https://github.com/eclipse-zenoh/zenoh-plugin-ros1   ROS2 zenoh bridge: https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds  But the topic remapping and throttling is better in swarm_ros_bridge.

## Introduction

A lightweight middle interface ROS package mainly based on [ZeroMQ](https://zeromq.org). It enables the specified ROS messages transmission among swarm robots through socket communication. The purpose of this package is to replace the traditional way of [running ROS across multiple machines in ROS1](https://wiki.ros.org/ROS/Tutorials/MultipleMachines), which has some drawbacks under swarm robots situation.

An example of two ROS robots communicating with each other through swarm_ros_bridge is shown below:

![framework](pictures/swarm_ros_bridge_framework.png)

Compared with ROS1 multi-robot wireless communication, it has the following benefits:

-  **Robust**: No need for base station ROS master launching first. Support each robot launching in a random sequence and connecting each other autonomously.

-  **Flexible**:  You can choose the sending/receiving ROS topics rather than transferring all topic (names) as ROS1 does.

-  **Easy to use**:  Specify all the IP and ROS topics in one configuration file.

Compared with ROS2 DDS communication, it may have the following benefits:

-  **Lightweight**: It is a small ROS bridge node subscribing and sending remote ROS topics, so connecting with other ROS nodes is easy.
-  **Reliable**: It uses ZeroMQ socket communication based on TCP protocol while ROS2 is based on DDS, whose default protocol is UDP (unreliable). DDS is mainly designed for data exchange between native processes under wired communication rather than remote wireless communication.

ROS wiki page: https://wiki.ros.org/swarm_ros_bridge

source code: https://github.com/shupx/swarm_ros_bridge.git

csdn blog (in chinese): https://blog.csdn.net/benchuspx/article/details/128576723


## Structure

```bash
└── swarm_ros_bridge
    ├── CMakeLists.txt
    ├── config
    │   └── ros_topics.yaml  # Config file to specify send/receive ROS topics
    ├── include
    │   ├── bridge_node.hpp  # Header file of bridge_node.cpp
    │   ├── ros_sub_pub.hpp  # Header file for different ROS message type.
    ├── launch
    │   └── test.launch
    ├── package.xml
    └── src
        └── bridge_node.cpp  # @brief Reliable TCP bridge for ros data transfer in unstable network.
                             # It will send/receive the specified ROS topics in ../config/ros_topics.yaml
                             # It uses zmq socket(PUB/SUB mode), which reconnects others autonomously and
                             # supports 1-N pub-sub connection even with TCP protocol.
```


## Install

**Supported platforms/releases**:

| Platform                                                   | ROS Release                                                    |
| ---------------------------------------------------------- | -------------------------------------------------------------- |
| [Ubuntu 16.04 Xenial](https://releases.ubuntu.com/16.04.4/) | [ROS Kinetic](https://wiki.ros.org/kinetic/Installation/Ubuntu) |
| [Ubuntu 18.04 Bionic](https://releases.ubuntu.com/18.04/) | [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu) |
| [Ubuntu 20.04 Focal](https://releases.ubuntu.com/20.04/) | [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) |

**Install process**:

```bash
## clone this package
mkdir -p swarm_ros_bridge_ws/src  # or your own ros workspace
cd swarm_ros_bridge_ws/src
git clone https://gitee.com/shu-peixuan/swarm_ros_bridge.git
# or 'git clone https://github.com/shupx/swarm_ros_bridge.git'

## install dependencies
sudo apt install libzmqpp-dev
# or 'rosdep install --from-path swarm_ros_bridge/'

## build
cd ../
catkin_make
source devel/setup.bash
```


## Usage

1. Specify the IP and ROS topic information in `config/ros_topics.yaml`. 

- For the sending topic, IP is self IP (* for example) and port should be different as it binds to the "tcp://*:port". 
- For the receiving topic, IP and port should be the remote source IP and port as it connects to the "tcp://srcIP:srcPort".

- The `max_freq` will limit the sending frequency once it exceeds `max_freq`. Set `max_freq` large enough if you do not want to decrease the sending frequency.

2. Launch the bridge_node:

```bash
roslaunch swarm_ros_bridge test.launch # local machine test
```

3. Publish messages into send_topics and check that remote recv_topics are receiving these messages. The console will also print INFO the first time recv_topics receive messages.


We also provide a simple latency test demo between two machines. Please refer to [scripts/README.md](https://github.com/shupx/swarm_ros_bridge/tree/master/scripts/README.md)


## Advanced

### * More ROS message types

The default supported ROS message types are `sensor_msgs/Imu`, `geometry_msgs/Twist`, `std_msgs/String`, `std_msgs/Float32`, and `std_msgs/UInt32`. If you need more types:

1. Modify the macros about MSG_TYPEx and MSG_CLASSx in `include/ros_sub_pub.hpp`, then it will generate template functions for different ros message types.  

```cpp
// In ros_sub_pub.hpp
// uncomment and modify the following lines:
#include <xxx_msgs/yy.h>
#define MSG_TYPE5 "xxx_msgs/yy"
#define MSG_CLASS5 xxx_msgs::yy
```

We support up to 10 types modification. If that is still not enough, then you should modify the `topic_subscriber()`, `topic_publisher()` and `deserialize_publish()` in `include/ros_sub_pub.hpp` according to their styles.

2. Add the dependent package in find_package() of `CMakeLists.txt`:

```sh
# in CMakeLists.txt
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  xxx_msgs
)
```

3. Recompile:

```bash
cd swarm_ros_bridge_ws/
catkin_make
```

### * More send_topics

We support up to 50 send_topics. Modify the following lines in `include/ros_sub_pub.hpp` if you need more.

```cpp
// in ros_sub_pub.hpp
# define SUB_MAX 50 // max number of subscriber callbacks
//...
template <typename T>
void (*sub_callbacks[])(const T &)=
{
  sub_cb<T,0>, sub_cb<T,1>, ... //add more
};

```

Then recompile:

```bash
cd swarm_ros_bridge_ws/
catkin_make
```

## Contributor

Peixuan Shu (shupeixuan@qq.com), PhD, beihang university, China, 2023.1.1

This package is personally developed by Peixuan Shu (PhD, beihang  university, China) on Jan. 2023. Any question/suggestion is welcomed at [swarm_ros_bridge issue](https://github.com/shupx/swarm_ros_bridge/issues) or [shupeixuan@qq.com](mailto:shupeixuan@qq.com). 

Some applications:

 **pyugvswarm** : a python package wrapper around unmanned ground vehicle (UGV) swarm positioning / communication / control ROS1 nodes. https://pyugvswarm.readthedocs.io/en/latest/overview.html

![img1](pictures/img1.png)

## Explicit platform observations

`bridge.launch` accepts the ordinary IP/send_topics/recv_topics configuration as
the `configuration` argument. The caller selects source and destination topics;
there is no ROS graph discovery or experiment-mode inference in this bridge.

In addition to the existing telemetry types, the compiled bridge supports
PoseStamped, TwistStamped, Path, MAVROS State/ExtendedState and ROS Clock.
Measurement pose and local-position estimate remain distinct topics. The
station visualizer derives rotor animation and bounded history; this bridge
does not calculate them or turn missing telemetry into predicted state.

Run `test/observation_containers_test.py --binary <bridge_node> --image <ROS image>
--result <result.json>` for two isolated container ROS masters. It checks the
explicit observation uplink, clock downlink, original timestamps and omission
of a private robot topic. Containers and the private test network are cleaned
up; evidence logs remain at the path in the result.
