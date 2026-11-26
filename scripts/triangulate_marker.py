#!/usr/bin/env python3
"""Detecta uma esfera verde e estima sua posição 3D por triangulação."""

import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from multi_camera_robot_localization.geometry import triangulate_rays


class MarkerTriangulation:
    """Combina os centros do marcador observados pelas duas câmeras."""

    def __init__(self):
        self.bridge = CvBridge()
        self.lower_hsv = np.asarray(rospy.get_param('~lower_hsv', [35, 80, 50]), dtype=np.uint8)
        self.upper_hsv = np.asarray(rospy.get_param('~upper_hsv', [95, 255, 255]), dtype=np.uint8)
        self.min_area = float(rospy.get_param('~min_area', 12.0))
        self.min_circularity = float(rospy.get_param('~min_circularity', 0.45))
        self.max_residual = float(rospy.get_param('~max_triangulation_residual', 0.25))
        self.cameras = []
        for index in (1, 2):
            self.cameras.append({
                'world_camera': np.asarray(
                    rospy.get_param('~camera_%d/world_camera' % index), dtype=float,
                ).reshape(4, 4),
                'info': None,
            })

        image_a = message_filters.Subscriber('/camera_1/image_raw', Image, queue_size=1)
        image_b = message_filters.Subscriber('/camera_2/image_raw', Image, queue_size=1)
        rospy.Subscriber('/camera_1/camera_info', CameraInfo, self.receive_info, callback_args=0)
        rospy.Subscriber('/camera_2/camera_info', CameraInfo, self.receive_info, callback_args=1)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [image_a, image_b], queue_size=5, slop=0.05,
        )
        self.sync.registerCallback(self.callback)
        self.publisher = rospy.Publisher('/multi_camera/pose', PoseStamped, queue_size=10)
        self.annotated_publishers = [
            rospy.Publisher('/camera_1/annotated', Image, queue_size=1),
            rospy.Publisher('/camera_2/annotated', Image, queue_size=1),
        ]

    def receive_info(self, message, camera_index):
        self.cameras[camera_index]['info'] = message

    def detect_center(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            circularity = 4.0 * np.pi * area / (perimeter * perimeter) if perimeter else 0.0
            if area >= self.min_area and circularity >= self.min_circularity:
                candidates.append(contour)
        if not candidates:
            return None, mask
        contour = max(candidates, key=cv2.contourArea)
        moments = cv2.moments(contour)
        if moments['m00'] <= 0.0:
            return None, mask
        center = (moments['m10'] / moments['m00'], moments['m01'] / moments['m00'])
        return center, mask

    def ray_from_pixel(self, pixel, camera_index):
        info = self.cameras[camera_index]['info']
        if info is None:
            raise ValueError('Calibração da câmera ainda não recebida')
        matrix = np.asarray(info.K, dtype=float).reshape(3, 3)
        distortion = np.asarray(info.D, dtype=float)
        normalized = cv2.undistortPoints(
            np.asarray([[pixel]], dtype=np.float32), matrix, distortion,
        ).reshape(2)
        direction_camera = np.array([normalized[0], normalized[1], 1.0], dtype=float)
        transform = self.cameras[camera_index]['world_camera']
        direction_world = transform[:3, :3].dot(direction_camera)
        direction_world /= np.linalg.norm(direction_world)
        return transform[:3, 3], direction_world

    def callback(self, image_a, image_b):
        images = [
            self.bridge.imgmsg_to_cv2(image_a, 'bgr8'),
            self.bridge.imgmsg_to_cv2(image_b, 'bgr8'),
        ]
        detections = []
        for image, publisher, message in zip(images, self.annotated_publishers, [image_a, image_b]):
            center, _ = self.detect_center(image)
            if center is not None:
                cv2.circle(image, tuple(int(value) for value in center), 12, (255, 0, 0), 2)
                cv2.putText(image, 'marcador verde', (20, 35), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (255, 0, 0), 2)
            annotated = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            annotated.header = message.header
            publisher.publish(annotated)
            detections.append(center)

        if detections[0] is None or detections[1] is None:
            return
        try:
            origin_a, direction_a = self.ray_from_pixel(detections[0], 0)
            origin_b, direction_b = self.ray_from_pixel(detections[1], 1)
            point, residual = triangulate_rays(origin_a, direction_a, origin_b, direction_b)
        except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
            rospy.logwarn_throttle(5.0, 'Triangulação inválida: %s', error)
            return
        if residual > self.max_residual:
            rospy.logwarn_throttle(5.0, 'Resíduo de triangulação alto: %.3f m', residual)
            return

        pose = PoseStamped()
        pose.header.stamp = max(image_a.header.stamp, image_b.header.stamp)
        pose.header.frame_id = 'world'
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]
        pose.pose.orientation.w = 1.0
        self.publisher.publish(pose)
        rospy.loginfo_throttle(
            2.0, 'Marcador 3D: x=%.3f y=%.3f z=%.3f resíduo=%.3f m',
            point[0], point[1], point[2], residual,
        )


if __name__ == '__main__':
    rospy.init_node('triangulate_marker')
    MarkerTriangulation()
    rospy.spin()
