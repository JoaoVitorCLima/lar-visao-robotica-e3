"""Geometria de uma detecção AprilTag em uma câmera calibrada."""

import math

import cv2
import numpy as np


def estimate_tag_on_plane(corners, camera_matrix, distortion, world_camera,
                          base_tag, tag_size, tag_height):
    """Estima a pose planar da tag e da base do robô no referencial `world`.

    A altura do plano da tag é um parâmetro mecânico calibrado. Ela não é
    obtida do ground truth do simulador nem ajustada pela trajetória.
    """
    half_size = float(tag_size) / 2.0
    object_points = np.array([
        [-half_size, half_size, 0.0],
        [half_size, half_size, 0.0],
        [half_size, -half_size, 0.0],
        [-half_size, -half_size, 0.0],
    ], dtype=np.float64)
    image_points = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    ok, rotation_vector, translation_vector = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok or translation_vector[2, 0] <= 0:
        raise ValueError('Invalid PnP solution')

    projected, _ = cv2.projectPoints(
        object_points,
        rotation_vector,
        translation_vector,
        camera_matrix,
        distortion,
    )
    residual = float(np.sqrt(np.mean(np.sum(
        (projected.reshape(4, 2) - image_points) ** 2,
        axis=1,
    ))))

    pixels = image_points.reshape(4, 1, 2)
    normalized = cv2.undistortPoints(pixels, camera_matrix, distortion).reshape(4, 2)
    rays = np.column_stack((normalized, np.ones(4))) @ world_camera[:3, :3].T
    origin = world_camera[:3, 3]
    if np.any(np.abs(rays[:, 2]) < 1e-8):
        raise ValueError('Camera ray parallel to tag plane')

    distance = (float(tag_height) - origin[2]) / rays[:, 2]
    if np.any(distance <= 0):
        raise ValueError('Plane behind camera')
    points = origin + distance[:, None] * rays
    tag_x_axis = (points[1] - points[0]) + (points[2] - points[3])
    angle = math.atan2(tag_x_axis[1], tag_x_axis[0])

    world_tag = np.eye(4)
    cosine, sine = math.cos(angle), math.sin(angle)
    world_tag[:3, :3] = [
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ]
    world_tag[:3, 3] = points.mean(axis=0)
    return world_tag @ np.linalg.inv(base_tag), residual
