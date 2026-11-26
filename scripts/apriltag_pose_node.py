#!/usr/bin/env python3
"""Detecta uma AprilTag e publica a pose planar do robô por câmera."""

import numpy as np
import cv2
import message_filters
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from tf.transformations import quaternion_from_matrix

from multi_camera_robot_localization.vision_geometry import estimate_tag_on_plane


class AprilTagPose:
    """Processa a imagem de uma câmera com calibração independente."""

    def __init__(self):
        self.bridge = CvBridge()
        self.tag_id = int(rospy.get_param('~tag_id', 42))
        self.tag_size = float(rospy.get_param('~tag_size', 0.40))
        self.tag_height = float(rospy.get_param('~tag_world_height', 0.742))
        self.max_reprojection_error = float(rospy.get_param('~max_reprojection_error', 2.0))
        self.world_camera = np.asarray(
            rospy.get_param('~world_camera'), dtype=float,
        ).reshape(4, 4)
        self.base_tag = np.eye(4)
        self.base_tag[:3, 3] = rospy.get_param('~tag_translation', [0.0, 0.0, 0.55])
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11,
        )
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        image_topic = rospy.get_param('~image_topic', '~image_raw')
        info_topic = rospy.get_param('~camera_info_topic', '~camera_info')
        pose_topic = rospy.get_param('~pose_topic', 'pose')
        annotated_topic = rospy.get_param('~annotated_topic', 'annotated')
        image = message_filters.Subscriber(image_topic, Image, queue_size=1)
        info = message_filters.Subscriber(info_topic, CameraInfo, queue_size=1)
        self.publisher = rospy.Publisher(pose_topic, PoseStamped, queue_size=10)
        self.annotated_publisher = rospy.Publisher(annotated_topic, Image, queue_size=1)
        self.sync = message_filters.ApproximateTimeSynchronizer([image, info], 5, 0.02)
        self.sync.registerCallback(self.callback)

    def callback(self, image_message, camera_info):
        image = self.bridge.imgmsg_to_cv2(image_message, 'bgr8')
        grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            grayscale,
            self.dictionary,
            parameters=self.parameters,
        )
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
        if ids is None:
            self.publish_annotated(image, image_message)
            return

        for marker_corners, marker_id in zip(corners, ids.ravel()):
            if int(marker_id) != self.tag_id:
                continue
            try:
                pose_matrix, residual = estimate_tag_on_plane(
                    marker_corners,
                    np.asarray(camera_info.K, dtype=float).reshape(3, 3),
                    np.asarray(camera_info.D, dtype=float),
                    self.world_camera,
                    self.base_tag,
                    self.tag_size,
                    self.tag_height,
                )
            except (ValueError, cv2.error) as error:
                rospy.logwarn_throttle(5.0, 'Detecção inválida: %s', error)
                continue

            if residual > self.max_reprojection_error:
                rospy.logwarn_throttle(
                    5.0,
                    'Erro de reprojeção acima do limite: %.3f px',
                    residual,
                )
                continue

            pose = PoseStamped()
            pose.header.stamp = image_message.header.stamp
            pose.header.frame_id = 'world'
            pose.pose.position.x = pose_matrix[0, 3]
            pose.pose.position.y = pose_matrix[1, 3]
            pose.pose.position.z = pose_matrix[2, 3]
            quaternion = quaternion_from_matrix(pose_matrix)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            self.publisher.publish(pose)
            cv2.putText(
                image,
                'AprilTag %d x=%.3f y=%.3f erro=%.2f px' %
                (self.tag_id, pose_matrix[0, 3], pose_matrix[1, 3], residual),
                (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2,
            )
            break
        self.publish_annotated(image, image_message)

    def publish_annotated(self, image, source_message):
        """Publica a imagem da câmera com a detecção desenhada."""
        annotated = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        annotated.header = source_message.header
        self.annotated_publisher.publish(annotated)


if __name__ == '__main__':
    rospy.init_node('apriltag_pose')
    AprilTagPose()
    rospy.spin()
