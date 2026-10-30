#!/usr/bin/env bash
set -eo pipefail

REPOSITORY_DIR="/home/minwoo/Development/LLM"
VENV_DIR="${REPOSITORY_DIR}/.yolo_venv"

/usr/bin/python3 -m venv --system-site-packages "${VENV_DIR}"
"${VENV_DIR}/bin/python" -m pip install --upgrade \
  "numpy<2" \
  "setuptools<80" \
  "opencv-python>=4.7,<4.13" \
  "torch==2.14.1" \
  "torchvision==0.29.1" \
  "ultralytics==8.4.173"

"${VENV_DIR}/bin/python" -c 'import cv2, numpy, rclpy, ultralytics; print("YOLO runtime ready:", ultralytics.__version__)'
