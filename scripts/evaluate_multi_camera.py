#!/usr/bin/env python3
"""Compara a pose fundida com o ground truth do Gazebo."""

import bisect
import csv
import os
import threading
from collections import deque

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

from multi_camera_robot_localization.geometry import wrap


def yaw(pose):
    """Extrai o yaw de uma orientação quaternion."""
    q = pose.orientation
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class Evaluation:
    """Registra erros temporais sem fornecer ground truth ao controlador."""

    def __init__(self):
        self.finished = False
        self.closed = False
        self.lock = threading.RLock()
        self.truth = deque(maxlen=2000)
        self.pending = deque(maxlen=100)
        path = rospy.get_param('~csv')
        self.ground_truth_z_offset = float(rospy.get_param('~ground_truth_z_offset', 0.72))
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.file = open(path, 'w', buffering=1)
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'time', 'x', 'y', 'z', 'yaw', 'gt_x', 'gt_y', 'gt_marker_z', 'gt_yaw',
            'error_position_m', 'error_position_3d_m', 'error_yaw_rad',
        ])
        rospy.Subscriber('/evaluation/ground_truth', Odometry, self.ground_truth, queue_size=100)
        rospy.Subscriber('/multi_camera/pose', PoseStamped, self.vision, queue_size=100)
        rospy.Subscriber('/multi_camera/done', Bool, self.done)
        rospy.on_shutdown(self.close)

    def ground_truth(self, message):
        with self.lock:
            if self.finished or self.closed:
                return
            self.truth.append((message.header.stamp.to_sec(), message.pose.pose))
            self.flush()

    def vision(self, message):
        with self.lock:
            if self.finished or self.closed:
                return
            self.pending.append(message)
            self.flush()

    def done(self, message):
        if message.data:
            with self.lock:
                if self.finished or self.closed:
                    return
                self.finished = True
                self.flush()
                self.pending.clear()
                self.file.flush()

    def close(self):
        """Fecha o CSV uma única vez e serializa o encerramento."""
        with self.lock:
            if self.closed:
                return
            self.finished = True
            self.flush()
            self.pending.clear()
            self.file.flush()
            self.file.close()
            self.closed = True

    def flush(self):
        if len(self.truth) < 2:
            return
        times = [item[0] for item in self.truth]
        while self.pending:
            message = self.pending[0]
            timestamp = message.header.stamp.to_sec()
            if timestamp > times[-1]:
                break
            self.pending.popleft()
            index = min(bisect.bisect_right(times, timestamp), len(times) - 1)
            if index == 0 or times[index] - times[index - 1] > 0.1:
                continue
            first, second = self.truth[index - 1][1], self.truth[index][1]
            fraction = (timestamp - times[index - 1]) / (times[index] - times[index - 1])
            ground_x = first.position.x + fraction * (second.position.x - first.position.x)
            ground_y = first.position.y + fraction * (second.position.y - first.position.y)
            ground_z = (first.position.z + fraction * (second.position.z - first.position.z)
                        + self.ground_truth_z_offset)
            ground_yaw = wrap(yaw(first) + fraction * wrap(yaw(second) - yaw(first)))
            pose = message.pose
            error_yaw = wrap(yaw(pose) - ground_yaw)
            error_position = ((pose.position.x - ground_x) ** 2 +
                              (pose.position.y - ground_y) ** 2) ** 0.5
            error_position_3d = ((pose.position.x - ground_x) ** 2 +
                                 (pose.position.y - ground_y) ** 2 +
                                 (pose.position.z - ground_z) ** 2) ** 0.5
            self.writer.writerow([
                timestamp, pose.position.x, pose.position.y, pose.position.z, yaw(pose),
                ground_x, ground_y, ground_z, ground_yaw,
                error_position, error_position_3d, error_yaw,
            ])


if __name__ == '__main__':
    rospy.init_node('evaluate_multi_camera')
    Evaluation()
    rospy.spin()
