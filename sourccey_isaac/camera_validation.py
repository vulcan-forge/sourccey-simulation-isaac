"""Check that the generated USD cameras preserve the source optical frames."""
import json
from pathlib import Path

import numpy as np
from pxr import UsdGeom
from scipy.spatial.transform import Rotation

from .camera_config import CAMERA_NAMES, CAMERA_PATHS, CAMERA_POSES
from .model import RobotModel


def validate(stage, path=None):
    model = RobotModel()
    report = {'cameras': {}}
    for name in CAMERA_NAMES:
        source = CAMERA_POSES[name]
        body = 'base_link' if source['body'] == 'robot_root' else source['body']
        group, body_in_group = model.original_frames[body]
        camera_in_body = np.eye(4)
        camera_in_body[:3, 3] = source['pos']
        w, x, y, z = source['quat_wxyz']
        camera_in_body[:3, :3] = Rotation.from_quat([x, y, z, w]).as_matrix()
        expected = body_in_group @ camera_in_body
        prim = stage.GetPrimAtPath(CAMERA_PATHS[name])
        if not prim.IsValid() or prim.GetTypeName() != 'Camera':
            raise AssertionError(f'Missing USD camera: {CAMERA_PATHS[name]}')
        if str(prim.GetParent().GetPath()) != f'/World/Sourccey/{group}':
            raise AssertionError(f'{name} is not mounted on {group}')
        actual_pos = np.array(prim.GetAttribute('xformOp:translate').Get())
        q = prim.GetAttribute('xformOp:orient').Get()
        actual_rot = Rotation.from_quat([*q.GetImaginary(), q.GetReal()]).as_matrix()
        position_error = float(np.linalg.norm(actual_pos - expected[:3, 3]))
        rotation_error = float(Rotation.from_matrix(actual_rot.T @ expected[:3, :3]).magnitude())
        camera = UsdGeom.Camera(prim)
        aperture = camera.GetVerticalApertureAttr().Get()
        focal = camera.GetFocalLengthAttr().Get()
        fovy = float(np.rad2deg(2 * np.arctan(aperture / (2 * focal))))
        fovy_error = abs(fovy - source['fovy_deg'])
        report['cameras'][name] = {'path': CAMERA_PATHS[name],
                                    'mount_position_error_m': position_error,
                                    'mount_rotation_error_rad': rotation_error,
                                    'fovy_error_deg': fovy_error}
    report['passed'] = all(v['mount_position_error_m'] < 1e-5 and
                           v['mount_rotation_error_rad'] < 1e-5 and
                           v['fovy_error_deg'] < 1e-4
                           for v in report['cameras'].values())
    if path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    if not report['passed']:
        raise AssertionError(f'USD camera frames differ from source: {report}')
    return report
