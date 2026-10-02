"""Capture the five USD-mounted simulated camera feeds in Isaac Sim.

Camera sensors are created one at a time to limit render-product memory on
machines with small GPUs. Capture after motion is complete: Replicator's
standalone capture step can invalidate a live PhysX articulation handle.
Import this module only after SimulationApp starts.
"""
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from .camera_config import CAMERA_NAMES, CAMERA_PATHS, CAMERA_SIZE


def capture_rgb(sim, name):
    """Return one 240x320 RGB frame from an existing robot-mounted camera."""
    if name not in CAMERA_PATHS:
        raise ValueError(f'Unknown camera {name!r}; choose from {CAMERA_NAMES}')
    from isaacsim.sensors.camera import Camera
    import omni.replicator.core as rep

    sensor = Camera(prim_path=CAMERA_PATHS[name], frequency=20, resolution=CAMERA_SIZE)
    sensor.initialize()
    try:
        rgb = None
        rep.orchestrator.set_capture_on_play(False)
        for _ in range(8):
            rep.orchestrator.step(delta_time=0.0, pause_timeline=False, rt_subframes=4)
            frame = sensor.get_rgba(device='cpu')
            if frame is not None and np.asarray(frame).size:
                rgb = np.asarray(frame)
                break
        if rgb is None:
            raise RuntimeError(f'No image arrived from {name} after 8 capture steps')
        expected = (CAMERA_SIZE[1], CAMERA_SIZE[0])
        if rgb.ndim == 2 and rgb.shape[0] == expected[0] * expected[1]:
            rgb = rgb.reshape(*expected, -1)
        if rgb.shape[:2] != expected or rgb.ndim != 3 or rgb.shape[2] < 3:
            raise RuntimeError(f'Invalid {name} camera frame shape: {rgb.shape}')
        return np.asarray(rgb[:, :, :3], dtype=np.uint8).copy()
    finally:
        sensor.destroy()


def save_tiled(sim, path):
    """Save the five cameras as a labeled 3x2 contact sheet."""
    width, height = CAMERA_SIZE
    label_height = 22
    image = Image.new('RGB', (width * 3, (height + label_height) * 2), (15, 18, 20))
    draw = ImageDraw.Draw(image)
    for index, name in enumerate(CAMERA_NAMES):
        x, y = index % 3 * width, index // 3 * (height + label_height)
        image.paste(Image.fromarray(capture_rgb(sim, name)), (x, y + label_height))
        draw.text((x + 6, y + 4), name.replace('_', ' '), fill=(100, 240, 150))
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)
    return path
