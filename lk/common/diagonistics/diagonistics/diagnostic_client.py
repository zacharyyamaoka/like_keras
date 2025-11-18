#!/usr/bin/env python3

""" Design Notes

    - it should create a subscription to the /diagonistic topic (accounting for same namespaces correctly)
    - it should listen to the diagonistics topic and store what it hears in a queue
    - it should optinally have lazy computation so by default it does nothing
    - When you actually ask for the level of diagonistics, it will 
    first find the index which is more recent than the current time minus the min(lifespan_sec or window_sec)
    It will then calculate the highest level of diagonistic in that window (OK, STALE, WARN, ERROR)
    - if the window is empty then it should return STALE
    - In the topic cb it should iterate through the queues starting from the front and remove items that are older than the lifespan. Once you hit
    and item that is not stale, you can stop iterating, as the remaining will be newer than the lifespan.

    For finding the highest level I can see the pattern used:
    https://github.com/ros/diagnostics/blob/ros2/diagnostic_updater/diagnostic_updater/_diagnostic_updater.py#L131

    Should use internall track timestamps or read it from header? is it from the time is was recivied that matters? or the time it was published?
"""

# ROS
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray
from rclpy.node import Node
from diagnostic_updater import DiagnosticStatusWrapper

# BAM
from ros_msg_factory.geometry_msgs import Time_from_sec, Time_to_sec
# PYTHON
import time
from collections import deque

class DiagonisticClient():
    
    def __init__(self, node: Node, lifespan_sec: int = 60, use_ros_clock=False):
        self.node = node
        self.lifespan_sec = lifespan_sec
        self.use_ros_clock = use_ros_clock
        self.clock = self.node.get_clock()
        self.start_time = time.time()

        self.msg_queue: deque = deque()
        self.stamp_queue: deque = deque()
        
        # Create subscription to diagnostic topic
        self.subscription = self.node.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.topic_cb,
            1
        )
    
    def now_sec(self):
        if self.use_ros_clock:
            return Time_to_sec(self.clock.now().to_msg())
        return time.time()

    def msg_stamp_sec(self, msg: DiagnosticArray):
        if self.use_ros_clock:
            return Time_to_sec(msg.header.stamp)
        return time.time()

    def topic_cb(self, msg: DiagnosticArray):        

        # Add new message to queues
        self.msg_queue.append(msg)
        self.stamp_queue.append(self.msg_stamp_sec(msg)) # better to append now then calculate each time you check...

        
        # Clean up old messages based on lifespan
        self._cleanup_old_messages()

    def _cleanup_old_messages(self):
        """Remove messages older than lifespan from the front of queues"""
        
        cutoff_time = self.now_sec() - self.lifespan_sec
        
        # Remove old messages from the front of all queues
        while (self.stamp_queue and self.stamp_queue[0] < cutoff_time):
            self.msg_queue.popleft()
            self.stamp_queue.popleft()

    def get_level(self):
        return self.get_status().level


    def get_status(self):
        """Calculate the highest diagnostic level in the specified window
        
        Based on     https://github.com/ros/diagnostics/blob/ros2/diagnostic_updater/diagnostic_updater/_diagnostic_updater.py#L131
        """
        if not self.msg_queue:
            return DiagnosticStatusWrapper(level=DiagnosticStatus.STALE)
        
        self._cleanup_old_messages()

        combined_summary = DiagnosticStatusWrapper()
        combined_summary.clearSummary()
        
        # Track the highest level across all statuses
        highest_level = DiagnosticStatus.OK
        
        # Process all messages in the queue
        for msg in self.msg_queue:
            stat: DiagnosticStatus
            for stat in msg.status:
                # Create wrapper using keyword arguments
                wrapped_status = DiagnosticStatusWrapper(
                    level=stat.level,
                    name=stat.name,
                    message=stat.message,
                    hardware_id=stat.hardware_id,
                    values=stat.values
                )
                
                combined_summary.mergeSummary(wrapped_status)

        return combined_summary

    @property
    def OK(self):
        return self.get_level() == DiagnosticStatus.OK

    @property
    def WARN(self):
        return self.get_level() == DiagnosticStatus.WARN

    @property
    def ERROR(self):
        return self.get_level() == DiagnosticStatus.ERROR

    @property
    def STALE(self):
        return self.get_level() == DiagnosticStatus.STALE


if __name__ == "__main__":
    rclpy.init()
    node = Node("scenario_client_server")
    diagonistic = DiagonisticClient(node)
    
    msg_1 = DiagnosticArray()
    msg_1.header.stamp = Time_from_sec(0.0) # stamp internal is done based on recivied time! so this won't matter...
    msg_1.status = [DiagnosticStatusWrapper(level=DiagnosticStatus.ERROR, name="status_1", message="error_status")]

    diagonistic.topic_cb(msg_1)

    msg_2 = DiagnosticArray()
    msg_2.header.stamp = node.get_clock().now().to_msg()
    msg_2.status = [DiagnosticStatusWrapper(level=DiagnosticStatus.OK, name="status_1", message="ok_status")]

    diagonistic.topic_cb(msg_2)
    diagonistic.topic_cb(msg_2)

    print(diagonistic.get_status().message)
    assert diagonistic.ERROR


    diagonistic = DiagonisticClient(node, use_ros_clock=True)

    diagonistic.topic_cb(msg_1)
    diagonistic.topic_cb(msg_2)
    diagonistic.topic_cb(msg_2)

    print(diagonistic.get_status().message)
    assert diagonistic.OK