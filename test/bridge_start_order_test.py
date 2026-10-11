#!/usr/bin/env python3
"""Real ROS transport and ZeroMQ peers, isolated from the machine's ROS master."""

import io
import json
import os
import random
import re
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import uuid
import xmlrpc.client

import rospy
import zmq
from std_msgs.msg import String


def free_port():
    for _ in range(100):
        port = random.randint(20000, 40000)
        with socket.socket() as sock:
            try:
                sock.bind(("127.0.0.1", port))
                return port
            except OSError:
                pass
    raise RuntimeError("No private test port available")


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
            raise AssertionError("Test child did not stop gracefully")
    return process.wait()


def master(port, configuration):
    # Preload parameters before opening the master socket, avoiding a test-only
    # race between native roscpp registration and rosparam configuration.
    from rosgraph.xmlrpc import XmlRpcNode
    from rosmaster.master_api import ROSMasterHandler
    # One callback worker keeps the master's per-thread XMLRPC connection cache
    # stable across publisher restarts when comparing total bridge descriptors.
    handler = ROSMasterHandler(num_workers=1)
    with open(configuration) as stream:
        handler.param_server.set_param("/", json.load(stream))
    stopped = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    node = XmlRpcNode(port, handler)
    node.start()
    try:
        stopped.wait()
    finally:
        node.shutdown("test complete")


def publisher(payload):
    rospy.init_node("late_publisher", disable_signals=False)
    pub = rospy.Publisher("/start_order/source", String, queue_size=1)
    while not rospy.is_shutdown():
        pub.publish(String(data=payload))
        time.sleep(0.05)


class BridgeStartOrderTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="bridge-start-order-")
        self.addCleanup(self.directory.cleanup)
        self.env = os.environ.copy()
        self.master_port, self.send_port, self.recv_port = free_port(), free_port(), free_port()
        self.uri = "http://127.0.0.1:{}".format(self.master_port)
        self.env.update(ROS_MASTER_URI=self.uri, ROS_HOME=self.directory.name,
                        ROS_IP="127.0.0.1", ROS_HOSTNAME="127.0.0.1")
        os.environ.update(self.env)
        configuration = {
            "IP": {"self": "127.0.0.1", "neighbor": "127.0.0.1"},
            "send_topics": [{"topic_name": "/start_order/source", "msg_type": "std_msgs/String",
                             "max_freq": 1000, "srcIP": "self", "srcPort": self.send_port}],
            "recv_topics": [{"topic_name": "/start_order/sink", "msg_type": "std_msgs/String",
                             "max_freq": 1000, "srcIP": "neighbor", "srcPort": self.recv_port}],
        }
        alias = dict(configuration["recv_topics"][0], topic_name="/start_order/alias")
        configuration["recv_topics"].append(alias)
        # Extra inactive topics must not create worker threads or TCP attempts.
        for index in range(3):
            configuration["recv_topics"].append({
                "topic_name": "/start_order/unused{}".format(index), "msg_type": "std_msgs/String",
                "max_freq": 1000, "srcIP": "neighbor", "srcPort": free_port()})
        self.config_path = os.path.join(self.directory.name, "parameters.json")
        with open(self.config_path, "w") as stream:
            json.dump({"start_order_bridge": configuration, "run_id": str(uuid.uuid4())}, stream)
        self.log_path = os.path.join(self.directory.name, "bridge.log")
        self.log = open(self.log_path, "w")
        self.addCleanup(self.log.close)
        self.children = []
        self.master_process = None
        self.addCleanup(self.stop_children)
        self.bridge = self.spawn([BRIDGE_BINARY, "__name:=start_order_bridge"], self.log)
        self.context = zmq.Context()
        self.addCleanup(self.context.term)
        self.peer = None
        self.addCleanup(self.close_peer)
        self.incoming = []
        self.incoming_lock = threading.Lock()

    def spawn(self, arguments, output=subprocess.DEVNULL):
        process = subprocess.Popen(arguments, env=self.env, stdout=output, stderr=subprocess.STDOUT)
        self.children.append(process)
        return process

    def stop_children(self):
        errors = []
        for child in reversed(self.children):
            if child is not self.master_process:
                try:
                    stop(child)
                except Exception as error:
                    errors.append(error)
        if self.master_process is not None:
            try:
                stop(self.master_process)
            except Exception as error:
                errors.append(error)
        if errors:
            raise AssertionError("Child cleanup failed: {}".format(errors))

    def logs(self):
        with open(self.log_path) as stream:
            return stream.read()

    def until(self, predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.assertIsNone(self.bridge.poll(), self.logs())
            if predicate():
                return
            time.sleep(0.05)
        self.fail("Condition timed out:\n" + self.logs())

    def close_peer(self):
        if self.peer is not None:
            self.peer.close(linger=0)
            self.peer = None

    def open_peer(self):
        self.peer = self.context.socket(zmq.XPUB)
        self.peer.setsockopt(zmq.XPUB_VERBOSE, 1)
        self.peer.setsockopt(zmq.SNDHWM, 1)
        self.peer.bind("tcp://127.0.0.1:{}".format(self.recv_port))
        self.until(lambda: bool(self.peer.poll(0)), timeout=35)
        self.assertEqual(self.peer.recv(), b"\x01", "Expected one subscription replay")

    def sample(self, payload):
        message = String(data=payload)
        stream = io.BytesIO()
        message.serialize(stream)
        data = stream.getvalue()
        self.peer.send_multipart([len(data).to_bytes(8 if sys.maxsize > 2**32 else 4,
                                                   "big"), data])

    def has_sample(self, payload):
        self.sample(payload)
        with self.incoming_lock:
            return payload in self.incoming

    def receive_ros(self, message):
        with self.incoming_lock:
            self.incoming.append(message.data)

    def metrics(self):
        self.assertIsNone(self.bridge.poll(), self.logs())
        root = "/proc/{}".format(self.bridge.pid)
        with open(root + "/stat") as stream:
            fields = stream.read().rsplit(")", 1)[1].split()
        cpu = (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")
        return cpu, len(os.listdir(root + "/task")), len(os.listdir(root + "/fd"))

    def outgoing(self, socket, payload):
        if not socket.poll(0):
            return False
        frames = socket.recv_multipart()
        self.assertEqual(len(frames), 2)
        self.assertEqual(int.from_bytes(frames[0], "big"), len(frames[1]))
        return String().deserialize(frames[1]).data == payload

    def test_parallel_start_and_recovery(self):
        time.sleep(1)
        self.assertIsNone(self.bridge.poll(), "Bridge exited before its master started")
        if STOP_WHILE_WAITING:
            self.assertEqual(stop(self.bridge), 0)
            return
        if MASTER_LONG_TEST:
            before = self.metrics()
            offset = len(self.logs())
            started = time.monotonic()
            time.sleep(60)
            after = self.metrics()
            cpu = after[0] - before[0]
            self.assertLess(cpu, 1.2)
            self.assertLessEqual(after[1], before[1])
            self.assertLessEqual(after[2], before[2])
            self.assertLessEqual(len(self.logs()[offset:].splitlines()), 7)
            print("master outage: seconds={:.2f} cpu={:.3f}s threads={}->{} fds={}->{} logs={}".format(
                time.monotonic() - started, cpu, before[1], after[1], before[2], after[2],
                len(self.logs()[offset:].splitlines())), flush=True)
        self.master_process = self.spawn(
            [sys.executable, __file__, "--master", str(self.master_port), self.config_path])
        proxy = xmlrpc.client.ServerProxy(self.uri)
        self.addCleanup(proxy.__exit__, None, None, None)

        def advertised():
            try:
                code, _, state = proxy.getSystemState("/start_order_test")
                return code == 1 and any(topic == "/start_order/sink" for topic, _ in state[0])
            except OSError:
                return False

        self.until(advertised, timeout=35)
        if MASTER_LONG_TEST:
            count = int(re.findall(r"ROS master bound attempts=(\d+)", self.logs())[-1])
            self.assertLessEqual(count, 15)
            print("master recovery: connect_attempts={}".format(count), flush=True)
        self.assertNotIn("Peer ", self.logs(), "Receiver connected before it had any ROS demand")
        rospy.init_node("start_order_test", disable_signals=True)
        self.addCleanup(rospy.signal_shutdown, "test complete")
        subscriber = rospy.Subscriber("/start_order/sink", String, self.receive_ros, queue_size=1)
        self.addCleanup(subscriber.unregister)
        self.until(lambda: "status=unavailable" in self.logs())
        # The remote bridge starts after both the node and its local subscriptions.
        self.open_peer()
        self.until(lambda: self.has_sample("remote-first"))
        alias_received = threading.Event()
        alias = rospy.Subscriber("/start_order/alias", String,
                                 lambda _: alias_received.set(), queue_size=1)
        self.addCleanup(alias.unregister)
        self.until(lambda: self.has_sample("remote-alias") and alias_received.is_set())
        self.assertFalse(self.peer.poll(200), "Duplicate connection to a shared peer")

        output = self.context.socket(zmq.SUB)
        output.setsockopt(zmq.SUBSCRIBE, b"")
        output.connect("tcp://127.0.0.1:{}".format(self.send_port))
        self.addCleanup(output.close, 0)
        pub = self.spawn([sys.executable, __file__, "--publisher", "publisher-first"])
        self.until(lambda: self.outgoing(output, "publisher-first"))
        self.assertEqual(stop(pub), 0)
        pub = self.spawn([sys.executable, __file__, "--publisher", "publisher-restarted"])
        self.until(lambda: self.outgoing(output, "publisher-restarted"))

        # No synthetic probes or traffic during the outage. Both a ROS publisher
        # and a ZeroMQ peer stay down while the bridge's own socket remains usable.
        if LONG_TEST:
            self.until(lambda: "status=connected" in self.logs(), timeout=11)
        baseline = self.metrics()
        self.assertEqual(stop(pub), 0)
        offset = len(self.logs())
        self.close_peer()
        before = self.metrics()
        started = time.monotonic()
        duration = 60 if LONG_TEST else 1
        time.sleep(duration)
        after = self.metrics()
        outage_logs = self.logs()[offset:]
        self.assertLessEqual(after[1], baseline[1], "Thread growth during outage")
        self.assertLessEqual(after[2], baseline[2], "File descriptor growth during outage")
        if LONG_TEST:
            cpu = after[0] - before[0]
            self.assertLess(cpu, 1.2, "Bridge exceeded 2 percent of one CPU core")
            self.assertLessEqual(len(outage_logs.splitlines()), 7)
            print("outage: seconds={:.2f} cpu={:.3f}s threads={}->{} fds={}->{} logs={}".format(
                time.monotonic() - started, cpu, baseline[1], after[1], baseline[2], after[2],
                outage_logs.count("Peer ")), flush=True)

        self.open_peer()
        with self.incoming_lock:
            self.incoming.clear()
        self.until(lambda: self.has_sample("remote-restarted"))
        # The static, empty-prefix subscription is replayed exactly once to the
        # new connection; this legacy data bridge owns no mutable remote state.
        self.assertFalse(self.peer.poll(500), "Repeated subscription replay")
        with self.incoming_lock:
            self.assertNotIn("remote-first", self.incoming, "Replayed stale sample backlog")
        self.spawn([sys.executable, __file__, "--publisher", "publisher-returned"])
        self.until(lambda: self.outgoing(output, "publisher-returned"))
        recovered = self.metrics()
        self.assertLessEqual(recovered[1], baseline[1])
        self.assertLessEqual(recovered[2], baseline[2])
        self.assertEqual(set(re.findall(r"Peer (\S+)", self.logs())), {
            "tcp://127.0.0.1:{}".format(self.recv_port)})
        if LONG_TEST:
            self.until(lambda: "status=connected" in self.logs()[offset:], timeout=11)
            summaries = re.findall(r"status=\w+ attempts=(\d+)", self.logs()[offset:])
            initial = re.findall(r"attempts=(\d+)", self.logs()[:offset])
            attempts = int(summaries[-1]) - int(initial[-1])
            self.assertLessEqual(attempts, 15, "Connect attempts exceeded backoff budget")
            print("recovery: connect_attempts={} subscription_replays=1 threads={} fds={} ROS_publisher=returned".format(
                attempts, recovered[1], recovered[2]), flush=True)
        if LONG_TEST:
            self.assertEqual(stop(self.master_process), 0)
            self.master_process = self.spawn(
                [sys.executable, __file__, "--master", str(self.master_port), self.config_path])
        else:
            proxy.setParam("/start_order_test", "/run_id", str(uuid.uuid4()))
        self.assertEqual(self.bridge.wait(timeout=12), 1, "Bridge served a stale master binding")
        self.assertIn("Bound ROS master lost or replaced", self.logs())


if __name__ == "__main__":
    if sys.argv[1:2] == ["--master"]:
        master(int(sys.argv[2]), sys.argv[3])
    elif sys.argv[1:2] == ["--publisher"]:
        publisher(sys.argv[2])
    else:
        LONG_TEST = "--long" in sys.argv
        MASTER_LONG_TEST = "--master-down-long" in sys.argv
        STOP_WHILE_WAITING = "--master-wait-stop" in sys.argv
        BRIDGE_BINARY = os.path.abspath(sys.argv[1])
        sys.argv = [sys.argv[0]]
        unittest.main()
