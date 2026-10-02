"""Move both arms to IK targets and record the five camera views."""
from pathlib import Path

import numpy as np

from sourccey_isaac.cameras import save_tiled


def run(sim, output):
    output = Path(output)
    targets = {}
    solver = {}
    for side in ('left', 'right'):
        target = sim.ee_local(side) + np.array([.025, 0, -.025])
        targets[side] = target
        solver[side] = sim.solve_ik(side, target)
    sim.step(900)
    errors = {side: float(np.linalg.norm(sim.ee_local(side) - target))
              for side, target in targets.items()}
    reached = save_tiled(sim, output / 'two_arm_reach.png')
    return {'example': 'two-arm-reach',
            'reach_error_m': {side: round(value, 4) for side, value in errors.items()},
            'solver_converged': {side: bool(solver[side]['converged']) for side in targets},
            'frame': str(reached),
            'passed': all(solver[s]['converged'] and errors[s] < .025 for s in targets)}
