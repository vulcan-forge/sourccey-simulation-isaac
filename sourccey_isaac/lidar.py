"""Forward planar lidar using PhysX collision-scene rays in Isaac Sim 5.1.

Import after SimulationApp starts. The 2D range format matches the MuJoCo port.
"""
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from PIL import Image, ImageDraw

CONFIG_PATH = Path(__file__).resolve().parents[1] / 'models/source/lidar_config.json'
CONFIG = json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
ANGLES = np.deg2rad(np.linspace(CONFIG['angle_min_deg'], CONFIG['angle_max_deg'],
                               CONFIG['beam_count']))


def scan(sim):
    """Return robot-relative front-aperture ranges; inf means no environmental hit."""
    import carb
    from omni.physx import get_physx_scene_query_interface

    position, quaternion = sim.robot.get_world_poses()
    root = Rotation.from_quat(quaternion[0][[1, 2, 3, 0]]).as_matrix()
    sensor = Rotation.from_quat(np.asarray(CONFIG['quaternion_body_wxyz'])[[1, 2, 3, 0]]).as_matrix()
    rotation = root @ sensor
    origin = position[0] + root @ np.asarray(CONFIG['position_body_m'])
    origin_carb = carb.Float3(*map(float, origin))
    query = get_physx_scene_query_interface()
    ranges = np.full(len(ANGLES), np.inf)
    hits = [None] * len(ANGLES)
    for i, angle in enumerate(ANGLES):
        direction = rotation @ [np.cos(angle), np.sin(angle), 0.]
        best = [float('inf'), None]

        def report(hit):
            body = str(hit.rigid_body)
            collision = str(hit.collision)
            if body.startswith('/World/Sourccey/') or collision.startswith('/World/Sourccey/'):
                return True
            distance = float(hit.distance)
            if CONFIG['range_min_m'] <= distance < best[0]:
                best[:] = [distance, collision or body]
            return True

        query.raycast_all(origin_carb, carb.Float3(*map(float, direction)),
                          float(CONFIG['range_max_m']), report)
        ranges[i], hits[i] = best
    return {'angles_rad': ANGLES.copy(), 'ranges_m': ranges, 'hit_names': hits,
            'origin_world_m': origin, 'rotation_world': rotation.copy(),
            'time_s': float(sim.elapsed)}


def as_dict(result):
    return {'frame_id': 'lidar_scan_origin',
            'sensor_model': CONFIG['sensor_model'],
            'angle_min_rad': float(ANGLES[0]),
            'angle_max_rad': float(ANGLES[-1]),
            'angle_increment_rad': float(ANGLES[1]-ANGLES[0]),
            'scan_rate_hz': CONFIG['scan_rate_hz'],
            'scan_time_s': 1.0 / CONFIG['scan_rate_hz'],
            'hardware_scan_deg': CONFIG['hardware_scan_deg'],
            'range_min_m': CONFIG['range_min_m'],
            'range_max_m': CONFIG['range_max_m'],
            'ranges_m': [float(x) if np.isfinite(x) else None for x in result['ranges_m']],
            'hit_names': result['hit_names'],
            'origin_world_m': result['origin_world_m'].tolist(),
            'time_s': result['time_s'],
            'calibration_status': CONFIG['calibration_status']}


def map_image(result, size=(800, 500)):
    """Robot-relative XY slice (+X up, +Y left), with collision returns."""
    width, height = size
    image = Image.new('RGB', size, (15, 20, 27))
    draw = ImageDraw.Draw(image)
    center = (width // 2, height - 45)
    view_range = min(CONFIG['range_max_m'], 2.5)
    scale = (height - 90) / view_range
    for radius_m in np.arange(.5, view_range + .01, .5):
        r = radius_m * scale
        draw.arc((center[0]-r, center[1]-r, center[0]+r, center[1]+r),
                 180, 360, fill=(48, 59, 71), width=1)
        draw.text((center[0]+5, center[1]-r-14), f'{radius_m:g} m', fill=(108, 130, 150))
    draw.line((20, center[1], width-20, center[1]), fill=(48, 59, 71), width=1)
    shown = 0
    for angle, distance in zip(ANGLES, result['ranges_m']):
        if np.isfinite(distance) and distance <= view_range:
            x = center[0] - np.sin(angle)*distance*scale
            y = center[1] - np.cos(angle)*distance*scale
            draw.ellipse((x-2, y-2, x+2, y+2), fill=(100, 245, 190))
            shown += 1
    draw.ellipse((center[0]-5, center[1]-5, center[0]+5, center[1]+5),
                 fill=(255, 150, 65))
    draw.text((12, 10), 'Sourccey virtual lidar | 180 degree XY slice', fill=(220, 233, 242))
    draw.text((12, 28), f'{shown} returns shown | view radius {view_range:g} m',
              fill=(145, 174, 190))
    draw.text((12, height-22), 'LEFT', fill=(145, 174, 190))
    draw.text((width-48, height-22), 'RIGHT', fill=(145, 174, 190))
    return image


def save_snapshot(sim, path):
    result = scan(sim)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(as_dict(result), indent=2, allow_nan=False)+'\n', encoding='utf-8')
    image_path = path.with_suffix('.png')
    map_image(result).save(image_path)
    return path, image_path
