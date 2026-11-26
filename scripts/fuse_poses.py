#!/usr/bin/env python3
"""Funde estimativas planares produzidas por câmeras já calibradas."""

import threading

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from multi_camera_robot_localization.geometry import fuse_planar_poses


class PoseFusion:
    """Nó que transforma várias estimativas temporais em uma pose única."""

    def __init__(self):
        self.lock = threading.RLock()
        self.poses = {}
        self.topics = rospy.get_param('~pose_topics', ['/camera_1/pose', '/camera_2/pose'])
        if isinstance(self.topics, str):
            self.topics = [self.topics]
        self.max_age = float(rospy.get_param('~max_pose_age', 0.5))
        self.publisher = rospy.Publisher('/multi_camera/pose', PoseStamped, queue_size=10)
        for topic in self.topics:
            rospy.Subscriber(topic, PoseStamped, self.receive, callback_args=topic, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_fused)

    def receive(self, message, topic):
        with self.lock:
            self.poses[topic] = message

    def publish_fused(self, _event):
        now = rospy.Time.now()
        with self.lock:
            current = [
                message for message in self.poses.values()
                if 0 <= (now - message.header.stamp).to_sec() <= self.max_age
            ]

        if not current:
            return

        planar_poses = []
        for message in current:
            orientation = message.pose.orientation
            yaw = euler_from_quaternion([
                orientation.x, orientation.y, orientation.z, orientation.w,
            ])[2]
            planar_poses.append((message.pose.position.x, message.pose.position.y, yaw))

        try:
            x, y, yaw = fuse_planar_poses(planar_poses)
        except ValueError as error:
            rospy.logwarn_throttle(5.0, 'Não foi possível fundir as poses: %s', error)
            return

        output = PoseStamped()
        output.header.stamp = max(message.header.stamp for message in current)
        output.header.frame_id = 'world'
        output.pose.position.x = x
        output.pose.position.y = y
        output.pose.position.z = 0.0
        output.pose.orientation.x, output.pose.orientation.y, output.pose.orientation.z, output.pose.orientation.w = \
            quaternion_from_euler(0.0, 0.0, yaw)
        self.publisher.publish(output)


if __name__ == '__main__':
    rospy.init_node('pose_fusion')
    PoseFusion()
    rospy.spin()
