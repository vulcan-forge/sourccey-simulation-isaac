"""Scan three reference obstacles with the lower-front planar lidar."""
from pathlib import Path

import numpy as np

from sourccey_isaac.lidar import ANGLES, CONFIG, save_snapshot, scan


def run(sim, output):
    sim.step(100)
    result = scan(sim)
    center = float(result['ranges_m'][int(np.argmin(np.abs(ANGLES)))])
    names = result['hit_names']
    sides = ('LeftBox' in str(names), 'RightBox' in str(names))
    data_path, image_path = save_snapshot(sim, Path(output) / 'lidar_demo.json')
    return {'example': 'lidar-demo', 'beam_count': len(names),
            'center_range_m': round(center, 4) if np.isfinite(center) else None,
            'left_detected': sides[0], 'right_detected': sides[1],
            'json': str(data_path), 'map': str(image_path),
            'passed': len(names) == CONFIG['beam_count'] and abs(center-.72) < .03 and all(sides)}
