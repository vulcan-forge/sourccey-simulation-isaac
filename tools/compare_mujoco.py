"""Run with MuJoCo's Python: verify the port's kinematics against MuJoCo."""
from pathlib import Path
import json
import sys
import numpy as np
import mujoco
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sourccey_isaac.model import RobotModel, SOURCE, MESH_ROOT, ROOT, WHEELS
import xml.etree.ElementTree as ET

source = ET.parse(SOURCE)
source.getroot().find('compiler').set('meshdir', str(MESH_ROOT))
m = mujoco.MjModel.from_xml_string(ET.tostring(source.getroot(), encoding='unicode'))
d = mujoco.MjData(m)
model = RobotModel()
assert len(model.joints) == 17
assert len(model.groups) == 18
assert np.isclose(sum(g['mass'] for g in model.groups.values()), m.body_mass.sum(), atol=1e-9)
rng = np.random.default_rng(413)
maximum = 0.
for sample in range(22):
    q = dict(model.defaults)
    if sample:
        for n, j in model.joints.items():
            q[n] = float(rng.uniform(-np.pi, np.pi) if n in WHEELS else rng.uniform(*j['limits']))
    mujoco.mj_resetDataKeyframe(m, d, 0)
    for name, value in q.items():
        d.qpos[m.joint(name).qposadr[0]] = value
    mujoco.mj_forward(m, d)
    frames = model.frames(q)
    for body, (group, transform) in model.original_frames.items():
        frame = frames[group] @ transform
        idx = m.body(body).id
        err = max(np.max(np.abs(frame[:3, 3]-d.xpos[idx])),
                  np.max(np.abs(frame[:3, :3]-d.xmat[idx].reshape(3, 3))))
        maximum = max(maximum, float(err))
        assert err < 1e-8, (sample, body, err)
    for side in ('left', 'right'):
        ee = model.point(side+'_ee', frames)
        assert np.allclose(ee, d.site_xpos[m.site(side+'_ee').id], atol=1e-8)
report = {'passed': True, 'poses': 22, 'original_bodies_compared': len(model.original_frames),
          'moving_bodies': len(model.groups), 'joints': len(model.joints),
          'mass_kg': float(m.body_mass.sum()), 'maximum_frame_error': maximum,
          'startup_pose_si': model.defaults}
(ROOT/'docs/kinematic_parity.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
