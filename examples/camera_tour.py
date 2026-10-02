"""Drive a short distance, move both wrists, and capture the five cameras."""
from pathlib import Path

import numpy as np

from sourccey_isaac.cameras import save_tiled


def run(sim, output):
    output = Path(output)
    sim.step(100)
    start = np.array(sim.state()['base_position'])
    sim.set_base(forward=.35)
    sim.step(650)
    sim.set_base()
    sim.step(150)
    sim.set_joints({'left_wrist_flex': .25, 'right_wrist_flex': -.25})
    sim.step(700)
    travel = float(np.linalg.norm(np.array(sim.state()['base_position'])[:2] - start[:2]))
    frame = save_tiled(sim, output / 'camera_tour.png')
    return {'example': 'camera-tour', 'base_travel_m': round(travel, 4),
            'frame': str(frame),
            'passed': travel > .03}
