"""PhysX articulation controller and contact-based mecanum traction."""
import numpy as np
from scipy.spatial.transform import Rotation
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation, RigidPrim
from .model import RobotModel, CAD_ROTATION, WHEELS
from .control import HandTarget, finite_vector, elevator_position, unity_arm, wheel_rates

DT = .002


class Simulation:
    def __init__(self, full_elevator_range=False, traction=True):
        self.description = RobotModel()
        self.defaults = self.description.defaults
        self.joint_limits = self.description.limits
        self.full_elevator_range, self.traction = full_elevator_range, traction
        self.world = World(physics_dt=DT, rendering_dt=1/60, stage_units_in_meters=1.0,
                           backend='numpy', device='cpu')
        self.robot = self.world.scene.add(Articulation('/World/Sourccey/base_link', name='sourccey',
                                                     reset_xform_properties=False))
        self.wheels = {}
        for name in WHEELS:
            body = self.description.joints[name]['child']
            view = RigidPrim('/World/Sourccey/'+body, name=name,
                             reset_xform_properties=False, track_contact_forces=True)
            self.world.scene.add(view)
            local = next(t[:3, 3] for g, t in self.description.groups[body]['geoms']
                         if g.get('name') == 'contact_'+name)
            self.wheels[name] = (view, local)
        self.world.reset()
        names = list(self.robot.dof_names)
        if set(names) != set(self.defaults):
            raise RuntimeError(f'Expected exactly 17 configured joints, got {names}')
        self.indices = {n: names.index(n) for n in names}
        self.position_names = [n for n in names if n not in WHEELS]
        self.position_ids = np.array([self.indices[n] for n in self.position_names])
        self.wheel_ids = np.array([self.indices[n] for n in WHEELS])
        self.force_caps = np.array([3000 if n == 'linear_actuator' else 30 if n in WHEELS else 100 for n in names])
        self.base_command = np.zeros(3)
        self.reset()

    def reset(self):
        # Reset the articulation directly; restarting all of Kit's render/physics
        # services here causes long pauses and unnecessary shader work.
        if not self.robot.is_physics_handle_valid():
            self.world.reset()
        root = self.description.root_transform
        quat = Rotation.from_matrix(root[:3, :3]).as_quat()[[3, 0, 1, 2]]
        self.robot.set_world_poses(positions=root[None, :3, 3], orientations=quat[None, :])
        self.robot.set_velocities(np.zeros((1, 6)))
        q = np.array([self.defaults[n] for n in self.robot.dof_names])
        self.robot.set_joint_positions(q[None, :])
        self.robot.set_joint_velocities(np.zeros((1, len(q))))
        self.robot.set_joint_efforts(np.zeros((1, len(q))))
        self.target_positions = dict(self.defaults)
        self.base_command[:] = 0
        self.set_base()
        self.set_joints({n: self.defaults[n] for n in self.position_names})
        self.targets = {side: HandTarget(self.description.reach(side, self.defaults), frozen=False)
                        for side in ('left', 'right')}
        self.elapsed = 0.0

    @property
    def robot_rotation(self):
        _, q = self.robot.get_world_poses()
        return Rotation.from_quat(q[0][[1, 2, 3, 0]]).as_matrix() @ CAD_ROTATION.T

    def positions(self):
        return dict(zip(self.robot.dof_names, self.robot.get_joint_positions()[0]))

    def ee_local(self, side):
        return self.description.reach(side, self.positions())

    def set_joints(self, values):
        pending = {}
        for name, value in values.items():
            if name not in self.position_names:
                raise ValueError(f'Unknown position-controlled joint: {name}')
            pending[name] = float(np.clip(finite_vector([value], 1)[0], *self.joint_limits[name]))
        self.target_positions.update(pending)
        targets = np.array([[self.target_positions[n] for n in self.position_names]])
        self.robot.set_joint_position_targets(targets, joint_indices=self.position_ids)

    def set_base(self, forward=0, left=0, yaw=0):
        self.base_command[:] = np.clip(finite_vector([forward, left, yaw], 3), -1, 1)
        self.robot.set_joint_velocity_targets(wheel_rates(self.base_command)[None, :], joint_indices=self.wheel_ids)

    def set_elevator(self, command):
        self.set_joints({'linear_actuator': elevator_position(command, self.full_elevator_range)})

    def set_unity_arm(self, side, degrees, closure=100):
        self.set_joints(unity_arm(side, degrees, closure))

    def solve_ik(self, side, target, iterations=60):
        if side not in ('left', 'right'):
            raise ValueError('side must be left or right')
        q, result = self.description.solve_ik(side, finite_vector(target, 3), self.target_positions, iterations)
        self.set_joints(q)
        return result

    def apply_traction(self):
        R = self.robot_rotation
        for i, name in enumerate(WHEELS):
            wheel, local = self.wheels[name]
            position, quaternion = wheel.get_world_poses()
            rotation = Rotation.from_quat(quaternion[0][[1, 2, 3, 0]]).as_matrix()
            center = position[0] + rotation @ local
            # The model's validated terrain is a flat, stationary plane at z=0.
            if center[2] > .058:
                continue
            contact = center-np.array([0., 0., .052])
            velocity = wheel.get_velocities()[0]
            com_local, _ = wheel.get_coms()
            com = position[0] + rotation @ np.asarray(com_local).reshape(-1, 3)[0]
            point_velocity = velocity[:3] + np.cross(velocity[3:], contact-com)
            direction = R @ np.array([1., (-1, 1, 1, -1)[i], 0.])
            direction[2] = 0
            direction /= np.linalg.norm(direction)
            normal = max(0., float(wheel.get_net_contact_forces(dt=DT)[0, 2]))
            force = np.clip(-1800*np.dot(direction, point_velocity), -.9*normal, .9*normal)*direction
            wheel.apply_forces_and_torques_at_pos(forces=force[None, :], positions=contact[None, :], is_global=True)

    def step(self, count=1):
        for _ in range(count):
            # Floating articulations can return six root-wrench entries before DOFs.
            # Isaac 5.1's wrapper otherwise truncates the floating-base buffer
            # to the first N entries, including the six root-wrench values.
            gravity = np.asarray(self.robot.get_generalized_gravity_forces(
                joint_indices=np.arange(len(self.indices))+6))[0].copy()
            gravity[self.wheel_ids] = 0
            self.robot.set_joint_efforts(np.clip(gravity, -self.force_caps, self.force_caps)[None, :])
            if self.traction:
                self.apply_traction()
            self.world.step(render=False)
            self.elapsed += DT

    def state(self):
        p, q = self.robot.get_world_poses()
        return {'time': self.elapsed, 'base_position': p[0].tolist(),
                'base_quaternion_wxyz': q[0].tolist(),
                'joints': {n: float(v) for n, v in self.positions().items()},
                'targets': {n: float(v) for n, v in self.target_positions.items()}}
