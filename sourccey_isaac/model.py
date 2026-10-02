"""Read the configured MuJoCo snapshot without importing MuJoCo.

Fixed CAD links are merged exactly: geometry, mass, centers of mass and inertia
are transformed into each moving body's frame. Joint coordinates stay in SI.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'models/reference/sourccey.xml'
MESH_ROOT = ROOT / 'models/source/SourcceyURDF'
USD = ROOT / 'models/sourccey.usdc'
ROLES = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
WHEELS = ('front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel')
ELEVATOR_UPPER_METERS = -0.0142
CAD_ROTATION = Rotation.from_euler('z', np.pi / 4).as_matrix()


def vector(text, default='0 0 0'):
    return np.fromstring(text or default, sep=' ')


def transform(element):
    out = np.eye(4)
    out[:3, 3] = vector(element.get('pos'))
    q = vector(element.get('quat'), '1 0 0 0')
    out[:3, :3] = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    return out


def motion(joint, value):
    out = np.eye(4)
    if joint['type'] == 'slide':
        out[:3, 3] = joint['axis'] * value
    else:
        out[:3, :3] = Rotation.from_rotvec(joint['axis'] * value).as_matrix()
    return out


class RobotModel:
    def __init__(self):
        xml = ET.parse(SOURCE).getroot()
        self.meshes = {e.get('name'): e for e in xml.findall('asset/mesh')}
        self.actuators = {e.get('name'): e for e in xml.findall('actuator/*')}
        controls = vector(xml.find('keyframe/key[@name="startup"]').get('ctrl'))
        self.defaults = dict(zip(self.actuators, controls))
        self.groups, self.joints, self.sites = {}, {}, {}
        self.original_frames = {}
        self.root_transform = transform(xml.find('worldbody/body'))

        def visit(body, group=None, relative=None):
            relative = np.eye(4) if relative is None else relative
            j = body.find('joint')
            if group is None or j is not None:
                parent = group
                group = body.get('name')
                self.groups[group] = {'geoms': [], 'inertials': []}
                if j is not None:
                    axis = vector(j.get('axis'))
                    axis /= np.linalg.norm(axis)
                    self.joints[j.get('name')] = {
                        'name': j.get('name'), 'parent': parent, 'child': group,
                        'origin': relative, 'axis': axis, 'type': j.get('type'),
                        'limits': vector(j.get('range'), '-inf inf'),
                    }
                relative = np.eye(4)
            self.original_frames[body.get('name')] = (group, relative.copy())
            inertial = body.find('inertial')
            if inertial is not None:
                vals = vector(inertial.get('fullinertia'))
                inertia = np.array([[vals[0], vals[3], vals[4]],
                                    [vals[3], vals[1], vals[5]],
                                    [vals[4], vals[5], vals[2]]])
                mass = float(inertial.get('mass'))
                com = relative[:3, :3] @ vector(inertial.get('pos')) + relative[:3, 3]
                inertia = relative[:3, :3] @ inertia @ relative[:3, :3].T
                self.groups[group]['inertials'].append((mass, com, inertia))
            for geom in body.findall('geom'):
                self.groups[group]['geoms'].append((geom, relative @ transform(geom)))
            for site in body.findall('site'):
                self.sites[site.get('name')] = (group, relative @ transform(site))
            for child in body.findall('body'):
                visit(child, group, relative @ transform(child))

        visit(xml.find('worldbody/body'))
        for group in self.groups.values():
            items = group['inertials']
            mass = sum(m for m, _, _ in items)
            com = sum(m * p for m, p, _ in items) / mass
            inertia = sum(I + m * (np.dot(p-com, p-com)*np.eye(3) - np.outer(p-com, p-com))
                          for m, p, I in items)
            group.update(mass=mass, com=com, inertia=inertia)
        self.limits = {n: j['limits'] for n, j in self.joints.items()}

    def frames(self, positions, root=None):
        frames = {'base_link': self.root_transform.copy() if root is None else root.copy()}
        for name, j in self.joints.items():
            frames[j['child']] = frames[j['parent']] @ j['origin'] @ motion(j, positions.get(name, 0.0))
        return frames

    def point(self, site, frames):
        group, local = self.sites[site]
        return (frames[group] @ local)[:3, 3]

    def reach(self, side, positions):
        frames = self.frames(positions, np.eye(4))
        return CAD_ROTATION @ (self.point(side+'_ee', frames) - self.point(side+'_mount', frames))

    def solve_ik(self, side, target, positions, iterations=60):
        q = dict(positions)
        names = [side+'_'+role for role in ROLES[:3]]
        for _ in range(iterations):
            f = self.frames(q, np.eye(4))
            end = self.point(side+'_ee', f)
            desired = self.point(side+'_mount', f) + CAD_ROTATION.T @ target
            error = desired-end
            norm = np.linalg.norm(error)
            if norm < .002:
                break
            columns = []
            for n in names:
                j = self.joints[n]
                jf = f[j['parent']] @ j['origin']
                columns.append(np.cross(jf[:3, :3] @ j['axis'], end-jf[:3, 3]))
            J = np.array(columns).T
            step = J.T @ np.linalg.solve(J@J.T + .015**2*np.eye(3), error*min(1, .08/norm))
            step = np.clip(step, -.15, .15)
            improved = False
            for scale in (1, .5, .25, .1):
                trial = dict(q)
                for n, delta in zip(names, step):
                    trial[n] = float(np.clip(q[n]+scale*delta, *self.limits[n]))
                if np.linalg.norm(self.reach(side, trial)-target) < norm:
                    q, improved = trial, True
                    break
            if not improved:
                break
        error = float(np.linalg.norm(self.reach(side, q)-target))
        return {n: q[n] for n in names}, {'error_m': error, 'converged': error < .002}


def mesh_points(mesh):
    path = MESH_ROOT / mesh.get('file')
    raw = path.read_bytes()
    count = int.from_bytes(raw[80:84], 'little')
    dtype = np.dtype([('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])
    points = np.frombuffer(raw, dtype=dtype, count=count, offset=84)['v'].reshape(-1, 3)
    return points * vector(mesh.get('scale'), '1 1 1')
