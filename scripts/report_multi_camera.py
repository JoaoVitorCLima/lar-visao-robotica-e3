#!/usr/bin/env python3
"""Gera métricas e gráficos a partir do CSV da avaliação multicâmera."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


parser = argparse.ArgumentParser()
parser.add_argument('csv', type=Path)
args = parser.parse_args()
data = np.atleast_1d(np.genfromtxt(args.csv, delimiter=',', names=True))
if len(data) < 2:
    raise SystemExit('At least two paired measurements required')

metrics = {
    'samples': len(data),
    'duration_s': float(data['time'][-1] - data['time'][0]),
    'method': 'detecção HSV de esfera verde e triangulação de dois raios',
}
for name, values in [
    ('position_m', data['error_position_m']),
    ('position_3d_m', data['error_position_3d_m']),
    ('yaw_deg', np.rad2deg(data['error_yaw_rad'])),
]:
    metrics[name] = {
        'rmse': float(np.sqrt(np.mean(values ** 2))),
        'mae': float(np.mean(abs(values))),
        'max_abs': float(np.max(abs(values))),
        'p95_abs': float(np.percentile(abs(values), 95)),
    }

(args.csv.parent / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')
figure, axes = plt.subplots(1, 4, figsize=(19, 4))
axes[0].plot(data['gt_x'], data['gt_y'], label='Gazebo')
axes[0].plot(data['x'], data['y'], label='Fusão multicâmera')
axes[0].axis('equal')
axes[0].set(xlabel='x [m]', ylabel='y [m]')
axes[0].legend()
time = data['time'] - data['time'][0]
axes[1].plot(time, data['error_position_m'])
axes[1].set(xlabel='tempo [s]', ylabel='erro de posição [m]')
axes[2].plot(time, np.rad2deg(data['error_yaw_rad']))
axes[2].set(xlabel='tempo [s]', ylabel='erro de yaw [graus]')
axes[3].plot(time, data['z'] - data['gt_marker_z'])
axes[3].set(xlabel='tempo [s]', ylabel='erro de altura [m]')
figure.tight_layout()
figure.savefig(args.csv.parent / 'errors.png', dpi=160)
print(json.dumps(metrics, indent=2))
