#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash

if [[ -f /ws/devel/setup.bash ]]; then
    source /ws/devel/setup.bash
fi

TAG_TEXTURE=/ws/src/multi_camera_robot_localization/models/tag42/materials/textures/tag42.png
if [[ ! -f "${TAG_TEXTURE}" ]]; then
    mkdir -p "$(dirname "${TAG_TEXTURE}")"
    python3 -c "import cv2; d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11); i=cv2.aruco.drawMarker(d,42,400); i=cv2.copyMakeBorder(i,50,50,50,50,cv2.BORDER_CONSTANT,value=255); assert cv2.imwrite('${TAG_TEXTURE}',i)"
fi

exec "$@"
