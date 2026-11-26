"""Operações geométricas independentes de ROS para fusão planar."""

import math
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np


Pose2D = Tuple[float, float, float]


def triangulate_rays(origin_a, direction_a, origin_b, direction_b):
    """Estima o ponto médio entre os pontos mais próximos de dois raios."""
    origin_a = np.asarray(origin_a, dtype=float).reshape(3)
    origin_b = np.asarray(origin_b, dtype=float).reshape(3)
    direction_a = np.asarray(direction_a, dtype=float).reshape(3)
    direction_b = np.asarray(direction_b, dtype=float).reshape(3)
    direction_a /= np.linalg.norm(direction_a)
    direction_b /= np.linalg.norm(direction_b)

    system = np.column_stack((direction_a, -direction_b))
    parameters, _, _, _ = np.linalg.lstsq(system, origin_b - origin_a, rcond=None)
    point_a = origin_a + parameters[0] * direction_a
    point_b = origin_b + parameters[1] * direction_b
    point = 0.5 * (point_a + point_b)
    residual = float(np.linalg.norm(point_a - point_b))
    if parameters[0] <= 0.0 or parameters[1] <= 0.0:
        raise ValueError('O marcador está atrás de uma das câmeras')
    return point, residual


def wrap(angle: float) -> float:
    """Normaliza um ângulo para o intervalo fechado [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def fuse_planar_poses(
    poses: Sequence[Pose2D],
    weights: Optional[Iterable[float]] = None,
) -> Pose2D:
    """Calcula a média ponderada de poses planares em um referencial comum.

    As coordenadas x e y são promediadas linearmente. O yaw é calculado pela
    média circular dos vetores unitários, evitando erros na transição entre -pi
    e pi. As poses precisam estar no mesmo referencial e representar o mesmo
    instante, ou instantes suficientemente próximos para a aplicação.
    """
    if not poses:
        raise ValueError('At least one pose is required')

    pose_values = [tuple(map(float, pose)) for pose in poses]
    if any(len(pose) != 3 for pose in pose_values):
        raise ValueError('Each pose must contain x, y and yaw')

    if weights is None:
        weight_values = [1.0] * len(pose_values)
    else:
        weight_values = [float(weight) for weight in weights]
        if len(weight_values) != len(pose_values):
            raise ValueError('Weights must match the number of poses')

    if any(not math.isfinite(weight) or weight <= 0 for weight in weight_values):
        raise ValueError('Weights must be finite and positive')

    total = sum(weight_values)
    x = sum(pose[0] * weight for pose, weight in zip(pose_values, weight_values)) / total
    y = sum(pose[1] * weight for pose, weight in zip(pose_values, weight_values)) / total
    sin_yaw = sum(math.sin(pose[2]) * weight for pose, weight in zip(pose_values, weight_values))
    cos_yaw = sum(math.cos(pose[2]) * weight for pose, weight in zip(pose_values, weight_values))
    if math.hypot(sin_yaw, cos_yaw) < 1e-12:
        raise ValueError('Yaw estimates are ambiguous')
    return x, y, wrap(math.atan2(sin_yaw, cos_yaw))
