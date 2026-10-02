"""Acceptance checks against actual PhysX state, not commanded targets."""
import json
import numpy as np
from scipy.spatial.transform import Rotation
from .simulation import DT
from .model import WHEELS


def validate(sim, output):
    report = {'backend': 'NVIDIA Isaac Sim / PhysX', 'checks': {}}
    def run(seconds):
        print(f'PHYSICS_CHECK: running {seconds:g} simulated seconds', flush=True)
        sim.step(round(seconds/DT))
        state = sim.state()
        assert np.all(np.isfinite(list(state['joints'].values()))), state
        return state
    try:
        sim.reset()
        assert len(sim.indices) == 17
        for name, value in sim.defaults.items():
            if name not in WHEELS:
                assert abs(sim.positions()[name]-value) < 1e-4, (name, sim.positions()[name], value)
        state = run(2.)
        errors = {n: abs(state['joints'][n]-v) for n, v in sim.defaults.items() if n not in WHEELS}
        report['checks']['startup_hold_errors'] = errors
        assert max(errors.values()) < .04, errors
        assert .06 < state['base_position'][2] < .18, state
        for name in ('left_elbow_flex', 'right_elbow_flex'):
            sim.reset(); sim.set_joints({name: -2.30})
            state = run(1.)
            error = abs(state['joints'][name]+2.30)
            report['checks'][name+'_lower_target_error'] = error
            assert error < .06, (name, error)
        sim.reset(); sim.set_elevator(-100); state = run(2.)
        report['checks']['elevator_bottom_m'] = state['joints']['linear_actuator']
        assert abs(state['joints']['linear_actuator']+.3104) < .008
        sim.set_elevator(100); state = run(2.)
        report['checks']['elevator_top_m'] = state['joints']['linear_actuator']
        assert abs(state['joints']['linear_actuator']+.0142) < .008
        for axis, command in (('forward', [.5, 0, 0]), ('left', [0, .5, 0]), ('yaw', [0, 0, .5])):
            sim.reset(); run(.5)
            before = sim.state()
            sim.set_base(*command); state = run(2.)
            delta = np.array(state['base_position'])-before['base_position']
            q = state['base_quaternion_wxyz']; q0 = before['base_quaternion_wxyz']
            r = Rotation.from_quat(np.array(q)[[1, 2, 3, 0]])
            r0 = Rotation.from_quat(np.array(q0)[[1, 2, 3, 0]])
            yaw = (r*r0.inv()).as_euler('xyz')[2]
            report['checks'][axis] = {'displacement_m': delta.tolist(), 'yaw_radians': float(yaw)}
            assert (delta[0] > .20 if axis == 'forward' else delta[1] > .20 if axis == 'left' else yaw > .3), report['checks'][axis]
            sim.set_base(); run(1.)
            report['checks'][axis]['stopped_velocity'] = sim.robot.get_velocities()[0].tolist()
            assert np.linalg.norm(sim.robot.get_velocities()[0]) < .15
        sim.reset(); run(.5)
        for side in ('left', 'right'):
            target = sim.ee_local(side)+[.015, 0, -.015]
            solution = sim.solve_ik(side, target)
            run(1.)
            error = np.linalg.norm(sim.ee_local(side)-target)
            report['checks'][side+'_reach'] = {'solver': solution, 'actual_error_m': float(error)}
            assert error < .02
        report['passed'] = True
    except Exception as exc:
        report['passed'] = False
        report['failure'] = repr(exc)
        raise
    finally:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2)+'\n')
        print('SOURCCEY_VALIDATION '+json.dumps(report), flush=True)
