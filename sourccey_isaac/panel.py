"""Native Isaac control panel, matching the MuJoCo joint and reach controls."""
import numpy as np
import omni.ui as ui
from .model import ROLES
from .camera_config import CAMERA_NAMES, CAMERA_PATHS


class Panel:
    def __init__(self, sim):
        self.sim = sim
        self.paused = False
        self.closed = False
        self.syncing = True
        self.ik_enabled = {side: False for side in ('left', 'right')}
        self.joint_models, self.reach_models = {}, {}
        self.base_models = []
        self.window = ui.Window('Sourccey | Controls', width=440, height=850)
        self.window.set_visibility_changed_fn(self.visibility_changed)
        viewport = ui.Workspace.get_window('Viewport')
        if viewport:
            self.window.dock_in(viewport, ui.DockPosition.RIGHT)
        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=5, height=0):
                    ui.Label('Sourccey - Isaac Sim', height=28)
                    ui.Label('Joint sliders: degrees | Elevator: meters', height=22)
                    with ui.HStack(height=28):
                        ui.Button('STOP BASE', clicked_fn=self.stop)
                        ui.Button('Reset pose', clicked_fn=self.reset)
                        ui.Button('Pause / resume', clicked_fn=self.pause)
                    with ui.CollapsableFrame('Simulated cameras (provisional poses)', collapsed=False):
                        with ui.VStack(spacing=3, height=0):
                            with ui.HStack(height=25):
                                ui.Button('Overview', clicked_fn=lambda: self.select_camera(None))
                                for name in CAMERA_NAMES[:2]:
                                    ui.Button(name.replace('_', ' '),
                                              clicked_fn=lambda n=name: self.select_camera(n))
                            with ui.HStack(height=25):
                                for name in CAMERA_NAMES[2:]:
                                    ui.Button(name.replace('_', ' '),
                                              clicked_fn=lambda n=name: self.select_camera(n))
                    with ui.CollapsableFrame('Base - release a slider to stop', collapsed=False):
                        with ui.VStack(spacing=3, height=0):
                            for axis in ('Forward', 'Left / strafe', 'CCW yaw'):
                                m = self.slider(axis, 0, -1, 1, lambda m: self.drive())
                                m.add_end_edit_fn(lambda m: self.stop())
                                self.base_models.append(m)
                    bottom = -.315 if sim.full_elevator_range else -.3104
                    self.elevator = self.slider('Linear actuator (m)', sim.defaults['linear_actuator'], bottom, -.0142,
                                                lambda m: self.elevator_changed(m))
                    for side in ('left', 'right'):
                        with ui.CollapsableFrame(side.title()+' arm', collapsed=False):
                            with ui.VStack(spacing=3, height=0):
                                for role in ROLES:
                                    name = side+'_'+role
                                    m = self.slider(role, np.rad2deg(sim.defaults[name]),
                                                    *np.rad2deg(sim.joint_limits[name]),
                                                    lambda m, n=name: self.joint_changed(n, m))
                                    self.joint_models[name] = m
                                with ui.CollapsableFrame('XYZ reach target (m)', collapsed=True):
                                    with ui.VStack(spacing=3, height=0):
                                        self.reach_models[side] = [
                                            self.slider(axis, value, -.5, .5, lambda m, s=side: self.enable_ik(s))
                                            for axis, value in zip('XYZ', sim.ee_local(side))]
                                        ui.Button('Freeze / resume target input', height=24,
                                                  clicked_fn=lambda s=side: self.freeze(s))
                    self.status = ui.Label('Ready - PhysX simulation', height=35, word_wrap=True)
                    ui.Button('Quit Sourccey', height=28, clicked_fn=self.close)
        self.syncing = False

    def slider(self, name, value, lower, upper, callback):
        with ui.HStack(height=26):
            ui.Label(name, width=160)
            widget = ui.FloatSlider(min=float(lower), max=float(upper), step=.001, format='%.3f')
            widget.model.set_value(float(value))
            widget.model.add_value_changed_fn(callback)
        return widget.model

    def drive(self):
        if not self.syncing:
            self.sim.set_base(*(m.as_float for m in self.base_models))

    def stop(self):
        self.sim.set_base()
        self.syncing = True
        for m in self.base_models:
            m.set_value(0.)
        self.syncing = False

    def elevator_changed(self, model):
        if not self.syncing:
            self.sim.set_joints({'linear_actuator': model.as_float})

    def joint_changed(self, name, model):
        if not self.syncing:
            self.ik_enabled[name.split('_')[0]] = False
            self.sim.set_joints({name: np.deg2rad(model.as_float)})

    def reset(self):
        self.stop(); self.sim.reset()
        self.syncing = True
        self.elevator.set_value(self.sim.defaults['linear_actuator'])
        for n, m in self.joint_models.items():
            m.set_value(float(np.rad2deg(self.sim.defaults[n])))
        for side in self.ik_enabled:
            self.ik_enabled[side] = False
            for m, v in zip(self.reach_models[side], self.sim.ee_local(side)):
                m.set_value(float(v))
        self.syncing = False
        self.status.text = 'Default pose restored'

    def pause(self):
        self.stop()
        self.paused = not self.paused
        self.status.text = 'Paused' if self.paused else 'Running'

    def select_camera(self, name):
        from isaacsim.core.utils.viewports import set_active_viewport_camera
        set_active_viewport_camera('/OmniverseKit_Persp' if name is None else CAMERA_PATHS[name])
        self.status.text = ('Overview camera' if name is None else name.replace('_', ' ') + ' camera')

    def enable_ik(self, side):
        if not self.syncing:
            self.ik_enabled[side] = True

    def freeze(self, side):
        target = self.sim.targets[side]
        if target.frozen:
            self.syncing = True
            for m, v in zip(self.reach_models[side], target.local):
                m.set_value(float(v))
            self.syncing = False
        target.frozen = not target.frozen
        self.status.text = side.title()+(' target frozen' if target.frozen else ' target active')

    def update(self):
        for side, enabled in self.ik_enabled.items():
            if not enabled:
                continue
            target = self.sim.targets[side]
            if not target.frozen:
                target.local[:] = [m.as_float for m in self.reach_models[side]]
            result = self.sim.solve_ik(side, target.local, iterations=8)
            self.syncing = True
            for role in ROLES[:3]:
                name = side+'_'+role
                self.joint_models[name].set_value(float(np.rad2deg(self.sim.target_positions[name])))
            self.syncing = False
            self.status.text = f"{side.title()} reach error {result['error_m']*1000:.1f} mm"

    def visibility_changed(self, visible):
        if not visible:
            self.stop()

    def close(self):
        self.stop()
        self.closed = True
