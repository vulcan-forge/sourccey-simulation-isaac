"""Shared five-camera calibration source; safe to import before SimulationApp."""
import json
from pathlib import Path


CAMERA_NAMES = ('front_left', 'front_right', 'bottom', 'wrist_left', 'wrist_right')
CAMERA_SOURCE = Path(__file__).resolve().parents[1] / 'models/source/camera_poses.json'
CAMERA_POSES = json.loads(CAMERA_SOURCE.read_text(encoding='utf-8'))
CAMERA_PATHS = {
    name: '/World/Sourccey/' +
    ('base_link' if CAMERA_POSES[name]['body'] == 'robot_root' else CAMERA_POSES[name]['body']) +
    '/camera_' + name
    for name in CAMERA_NAMES
}
CAMERA_SIZE = (320, 240)
