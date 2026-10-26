#!/usr/bin/env python3

import os
import random
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import xmlrpc.client

import rospy
from std_msgs.msg import String


BRIDGE_BINARY = None


def find_free_port():
    for _ in range(100):
        port = random.randint(20000, 40000)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            try:
                sock.bind(("127.0.0.1", port))
                return port
            except OSError:
                pass
    raise RuntimeError("No private ROS test port available")


def wait_for_master(uri, timeout=10.0):
    master = xmlrpc.client.ServerProxy(uri)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            code, _, _ = master.getPid("/bridge_lifecycle_test")
            if code == 1:
                return
        except OSError:
            pass
        time.sleep(0.05)
    raise RuntimeError("ROS master did not start")


def stop_process(process, timeout=10.0):
    if process is None or process.poll() is not None:
        return process.returncode if process is not None else None
    process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2.0)
        raise


class BridgeLifecycleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ros_home = tempfile.TemporaryDirectory(prefix="bridge-lifecycle-")
        cls.addClassCleanup(cls.ros_home.cleanup)
        cls.master_port = find_free_port()
        cls.topic_port = find_free_port()
        cls.master_uri = "http://127.0.0.1:{}".format(cls.master_port)
        cls.env = os.environ.copy()
        cls.env.update({
            "ROS_MASTER_URI": cls.master_uri,
            "ROS_IP": "127.0.0.1",
            "ROS_HOSTNAME": "127.0.0.1",
            "ROS_HOME": cls.ros_home.name,
        })
        os.environ.update(cls.env)
        cls.roscore = subprocess.Popen(
            ["roscore", "-p", str(cls.master_port)],
            env=cls.env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        cls.addClassCleanup(stop_process, cls.roscore)
        wait_for_master(cls.master_uri)

        rospy.init_node("bridge_lifecycle_test", anonymous=True, disable_signals=True)
        rospy.set_param("/bridge_lifecycle_bridge/IP", {
            "self": "127.0.0.1",
            "neighbor": "127.0.0.1",
        })
        rospy.set_param("/bridge_lifecycle_bridge/send_topics", [{
            "topic_name": "/bridge_lifecycle/chatter",
            "msg_type": "std_msgs/String",
            "max_freq": 1000,
            "srcIP": "self",
            "srcPort": cls.topic_port,
        }])
        rospy.set_param("/bridge_lifecycle_bridge/recv_topics", [{
            "topic_name": "/bridge_lifecycle/chatter_recv",
            "msg_type": "std_msgs/String",
            "max_freq": 1000,
            "srcIP": "neighbor",
            "srcPort": cls.topic_port,
        }])

        cls.bridge = subprocess.Popen(
            [BRIDGE_BINARY, "__name:=bridge_lifecycle_bridge"],
            env=cls.env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        cls.addClassCleanup(stop_process, cls.bridge)

    @classmethod
    def tearDownClass(cls):
        if getattr(cls, "bridge", None) is not None and cls.bridge.poll() is None:
            stop_process(cls.bridge)
        if rospy.core.is_initialized():
            rospy.signal_shutdown("test complete")
        if getattr(cls, "roscore", None) is not None:
            stop_process(cls.roscore)

    def test_message_round_trip_and_clean_shutdown(self):
        received = threading.Event()
        payload = "bridge-lifecycle-asan"

        def callback(message):
            if message.data == payload:
                received.set()

        subscriber = rospy.Subscriber(
            "/bridge_lifecycle/chatter_recv", String, callback, queue_size=1
        )
        publisher = rospy.Publisher(
            "/bridge_lifecycle/chatter", String, queue_size=1)

        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not received.is_set():
            if self.bridge.poll() is not None:
                output = self.bridge.stdout.read().decode("utf-8", errors="replace")
                self.fail("bridge exited before forwarding a message:\n{}".format(output))
            publisher.publish(String(data=payload))
            received.wait(0.1)

        subscriber.unregister()
        self.assertTrue(received.is_set(), "bridge did not forward the test message")

        returncode = stop_process(self.bridge)
        type(self).bridge = None
        self.assertEqual(returncode, 0, "bridge did not exit cleanly")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: bridge_lifecycle_test.py /path/to/bridge_node")
    BRIDGE_BINARY = os.path.abspath(sys.argv[1])
    sys.argv = [sys.argv[0]]
    unittest.main()
