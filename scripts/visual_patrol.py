#!/usr/bin/env python3
"""Move o Husky usando somente a pose fundida das câmeras."""

import math

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

from multi_camera_robot_localization.geometry import wrap


class Patrol:
    """Controlador proporcional simples para os pontos de teste."""

    def __init__(self):
        self.pose = None
        self.index = 0
        self.points = rospy.get_param(
            '~waypoints', [[4.8, 3.0], [4.8, 4.0], [3.8, 4.0], [3.8, 3.0]],
        )
        self.publisher = rospy.Publisher(
            '/husky_velocity_controller/cmd_vel', Twist, queue_size=1,
        )
        self.done = rospy.Publisher('/multi_camera/done', Bool, queue_size=1, latch=True)
        self.done.publish(False)
        rospy.Subscriber('/multi_camera/pose', PoseStamped, self.receive)
        rospy.on_shutdown(lambda: self.publisher.publish(Twist()))
        self.timer = rospy.Timer(rospy.Duration(0.05), self.step)

    def receive(self, pose):
        self.pose = pose

    def step(self, _event):
        command = Twist()
        if self.index >= len(self.points):
            self.done.publish(True)
        elif self.pose is not None and 0 <= (rospy.Time.now() - self.pose.header.stamp).to_sec() < 0.5:
            point = self.pose.pose
            target_x, target_y = self.points[self.index]
            dx, dy = target_x - point.position.x, target_y - point.position.y
            orientation = point.orientation
            yaw = euler_from_quaternion([
                orientation.x, orientation.y, orientation.z, orientation.w,
            ])[2]
            error = wrap(math.atan2(dy, dx) - yaw)
            distance = math.hypot(dx, dy)
            if distance < 0.12:
                self.index += 1
                rospy.loginfo('Waypoint %s/%s', self.index, len(self.points))
            else:
                command.angular.z = max(-0.5, min(0.5, 1.5 * error))
                if abs(error) < 0.35:
                    command.linear.x = min(0.2, 0.6 * distance)
        self.publisher.publish(command)


if __name__ == '__main__':
    rospy.init_node('visual_patrol')
    Patrol()
    rospy.spin()
