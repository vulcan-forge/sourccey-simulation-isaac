"""Two visible assisted pickups with PhysX parcels and IK-driven arms.

Each parcel is held at its measured palm-relative pose during the carry, then
returned to dynamic PhysX motion. Finger contact alone is not validated.
"""
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

PARCEL_START = {'left': np.array([.63, .18, .595]),
                'right': np.array([.63, -.18, .595])}


class WorkshopDemo:
    def __init__(self, sim):
        self.sim = sim
        self.phase = -1
        self.phase_time = 0.
        self.next_control = 0.
        self.move_start = {}
        self.move_target = {}
        self.held = {}
        self.peak_lift = {side: 0. for side in PARCEL_START}
        self.closest_approach = {side: float('inf') for side in PARCEL_START}
        self.release_xy = {}
        self.phases = (
            (0.0, 'Settling by the pickup table', self.stop),
            (.3, 'Rolling up to the two parcels', self.approach),
            (1.3, 'Opening both grippers', self.open_hands),
            (1.7, 'Reaching above the parcels', self.above),
            (3.3, 'Lowering both hands beside the parcels', self.lower),
            (5.3, 'Closing the grippers', self.close_hands),
            (5.9, 'Assisted attachment at the palms', self.attach),
            (6.2, 'Lifting both parcels off the table', self.lift),
            (8.0, 'Carrying parcels to the colored pads', self.carry_out),
            (9.6, 'Lowering parcels over their pads', self.place),
            (11.4, 'Opening grippers over the landing pads', self.open_for_release),
            (12.1, 'Releasing both parcels into PhysX', self.release),
            (12.8, 'Retracting both arms', self.retract),
            (14.6, 'Inspecting the placed parcels', self.stop),
            (15.6, 'Pickup demo complete', self.stop),
        )

    @property
    def done(self):
        return self.phase == len(self.phases) - 1

    def root_frame(self):
        state = self.sim.state()
        q = np.asarray(state['base_quaternion_wxyz'])
        out = np.eye(4)
        out[:3, 3] = state['base_position']
        out[:3, :3] = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
        return out

    def frames(self):
        return self.sim.description.frames(self.sim.positions(), self.root_frame())

    def palm_frame(self, side):
        name = 'Gripper_Base_v1_1' if side == 'left' else 'Gripper_Base_v1_2'
        group, relative = self.sim.description.original_frames[name]
        return self.frames()[group] @ relative

    def local_target(self, side, world):
        mount = self.sim.description.point(side+'_mount', self.frames())
        return self.sim.robot_rotation.T @ (np.asarray(world)-mount)

    def set_move(self, height, outward=0.):
        self.move_start = {side: self.sim.ee_local(side).copy() for side in PARCEL_START}
        self.move_target = {}
        for side, parcel in PARCEL_START.items():
            y = parcel[1] + (outward if side == 'left' else -outward)
            self.move_target[side] = self.local_target(side, [.60, y, height])
        self.next_control = self.sim.elapsed

    def stop(self):
        self.move_target = {}
        self.sim.set_base()

    def approach(self):
        self.move_target = {}
        self.sim.set_base(forward=.20)

    def open_hands(self):
        self.stop()
        self.sim.set_joints({side+'_gripper': np.deg2rad(30)
                             for side in PARCEL_START})

    def above(self):
        self.set_move(.80)

    def lower(self):
        self.set_move(.69)

    def close_hands(self):
        self.move_target = {}
        self.sim.set_joints({side+'_gripper': self.sim.defaults[side+'_gripper']
                             for side in PARCEL_START})

    def attach(self):
        self.move_target = {}
        from pxr import UsdPhysics
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        for side, prop in self.sim.props.items():
            parcel_pos, parcel_q = prop.get_world_poses()
            parcel_pos, parcel_q = parcel_pos[0], parcel_q[0]
            palm = self.palm_frame(side)
            ee = self.sim.description.point(side+'_ee', self.frames())
            distance = float(np.linalg.norm(ee-parcel_pos))
            self.closest_approach[side] = distance
            # EE is the wrist site; finger tips extend below it toward the cube.
            if distance > .18:
                raise RuntimeError(f'{side} hand missed parcel by {distance:.3f} m; '
                                   f'ee={ee}, parcel={parcel_pos}, base={self.sim.state()["base_position"]}')
            offset_pos = palm[:3, :3].T @ (parcel_pos-palm[:3, 3])
            offset_rot = palm[:3, :3].T @ Rotation.from_quat(
                parcel_q[[1, 2, 3, 0]]).as_matrix()
            self.held[side] = (offset_pos, offset_rot)
            path = '/World/PickupDemo/'+side.title()+'Parcel'
            UsdPhysics.RigidBodyAPI.Get(stage, path).GetKinematicEnabledAttr().Set(True)

    def lift(self):
        self.set_move(.87)

    def carry_out(self):
        self.set_move(.87, outward=.13)

    def place(self):
        self.set_move(.70, outward=.13)

    def open_for_release(self):
        self.move_target = {}
        self.sim.set_joints({side+'_gripper': np.deg2rad(30)
                             for side in PARCEL_START})

    def release(self):
        self.move_target = {}
        from pxr import UsdPhysics
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        for side, prop in self.sim.props.items():
            if side not in self.held:
                continue
            self.release_xy[side] = prop.get_world_poses()[0][0][:2].copy()
            path = '/World/PickupDemo/'+side.title()+'Parcel'
            UsdPhysics.RigidBodyAPI.Get(stage, path).GetKinematicEnabledAttr().Set(False)
        self.held.clear()

    def retract(self):
        self.move_start = {side: self.sim.ee_local(side).copy() for side in PARCEL_START}
        self.move_target = {side: self.sim.description.reach(side, self.sim.defaults)
                            for side in PARCEL_START}
        self.next_control = self.sim.elapsed

    def update(self):
        next_phase = self.phase + 1
        if next_phase < len(self.phases) and self.sim.elapsed >= self.phases[next_phase][0]:
            self.phase = next_phase
            self.phase_time = self.sim.elapsed
            _, caption, command = self.phases[next_phase]
            command()
            return caption
        if self.move_target and self.sim.elapsed >= self.next_control:
            end = self.phases[min(self.phase + 1, len(self.phases)-1)][0]
            span = max(.001, end-self.phase_time)
            alpha = np.clip((self.sim.elapsed-self.phase_time)/(span*.65), 0., 1.)
            alpha = alpha*alpha*(3.-2.*alpha)
            for side in PARCEL_START:
                target = self.move_start[side] + alpha*(self.move_target[side]-self.move_start[side])
                result = self.sim.solve_ik(side, target)
                if result['error_m'] > .025:
                    raise RuntimeError(f'{side} IK missed {self.phases[self.phase][1]}: {result}')
            self.next_control = self.sim.elapsed + .02
        return None

    def post_step(self):
        for side, (offset_pos, offset_rot) in self.held.items():
            palm = self.palm_frame(side)
            pos = palm[:3, 3] + palm[:3, :3] @ offset_pos
            rot = palm[:3, :3] @ offset_rot
            q = Rotation.from_matrix(rot).as_quat()[[3, 0, 1, 2]]
            prop = self.sim.props[side]
            prop.set_world_poses(positions=pos[None, :], orientations=q[None, :])
            self.peak_lift[side] = max(self.peak_lift[side],
                                       float(pos[2]-PARCEL_START[side][2]))

    def reset_props(self):
        from pxr import UsdPhysics
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        for side, prop in self.sim.props.items():
            path = '/World/PickupDemo/'+side.title()+'Parcel'
            UsdPhysics.RigidBodyAPI.Get(stage, path).GetKinematicEnabledAttr().Set(False)
            prop.set_world_poses(positions=PARCEL_START[side][None, :],
                                 orientations=np.array([[1., 0., 0., 0.]]))
            prop.set_velocities(np.zeros((1, 6)))

    def resume(self):
        if self.phase == 1:
            self.sim.set_base(forward=.20)


def run(sim, output):
    from sourccey_isaac.cameras import save_tiled

    demo = WorkshopDemo(sim)
    while not demo.done:
        demo.update()
        sim.step()
        demo.post_step()
    sim.step(200)
    positions = {side: sim.props[side].get_world_poses()[0][0].tolist()
                 for side in PARCEL_START}
    report = {'example': 'workshop-demo', 'grasp_mode': 'assisted kinematic carry',
              'approach_m': demo.closest_approach,
              'peak_lift_m': demo.peak_lift,
              'release_xy_m': {side: xy.tolist() for side, xy in demo.release_xy.items()},
              'final_positions_m': positions,
              'simulated_seconds': round(sim.elapsed, 3)}
    report['passed'] = all(demo.closest_approach[s] < .18 and demo.peak_lift[s] > .10
                           and abs(demo.release_xy[s][1]-PARCEL_START[s][1]) > .07
                           and .53 < positions[s][2] < .68 for s in PARCEL_START)
    frame = save_tiled(sim, Path(output) / 'workshop_demo.png')
    report['frame'] = str(frame)
    return report
