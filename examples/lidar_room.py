"""Live LD19 room traversal with a native Isaac map inset and 3D scan rays."""
import json
from pathlib import Path

import numpy as np
from PIL import Image

from sourccey_isaac.lidar import ANGLES, CONFIG, map_image, save_snapshot, scan


PHASES = (
    (.5, 'Settling in the room', (0, 0, 0)),
    (3.7, 'Approaching the first obstacle pair', (.55, 0, 0)),
    (4.6, 'Sliding left through the aisle', (0, .5, 0)),
    (7.8, 'Passing the colored crates', (.55, 0, 0)),
    (8.7, 'Sliding back toward center', (0, -.5, 0)),
    (11.2, 'Approaching the far wall', (.5, 0, 0)),
    (13.0, 'Turning to scan the room', (0, 0, .35)),
    (13.5, 'Room scan complete', (0, 0, 0)),
)


class ScanFan:
    """Render-only USD markers; they cannot affect collisions or lidar hits."""
    def __init__(self):
        import omni.usd
        from pxr import Gf, UsdGeom

        stage = omni.usd.get_context().get_stage()
        UsdGeom.Xform.Define(stage, '/World/LidarRoom/ScanFan')
        self.rays = []
        self.indices = list(range(0, len(ANGLES), 15))
        for index in self.indices:
            ray = UsdGeom.Cylinder.Define(stage, f'/World/LidarRoom/ScanFan/Ray{index:03d}')
            ray.CreateAxisAttr(UsdGeom.Tokens.x)
            ray.CreateHeightAttr(1.)
            ray.CreateRadiusAttr(.006)
            ray.CreateDisplayColorAttr([Gf.Vec3f(.04, .95, .82)])
            translate = ray.AddTranslateOp()
            rotate = ray.AddRotateZOp()
            scale = ray.AddScaleOp()
            self.rays.append((translate, rotate, scale))
        marker = UsdGeom.Sphere.Define(stage, '/World/LidarRoom/ScanFan/Origin')
        marker.CreateRadiusAttr(.007)
        marker.CreateDisplayColorAttr([Gf.Vec3f(1., .26, .08)])
        self.marker = marker.AddTranslateOp()

    def update(self, reading):
        from pxr import Gf

        origin = reading['origin_world_m']
        rotation = reading['rotation_world']
        self.marker.Set(Gf.Vec3d(*map(float, origin)))
        for index, (translate, rotate, scale) in zip(self.indices, self.rays):
            angle = ANGLES[index]
            direction = rotation @ [np.cos(angle), np.sin(angle), 0.]
            length = min(float(reading['ranges_m'][index]), 2.2)
            midpoint = origin + direction * (length * .5)
            translate.Set(Gf.Vec3d(*map(float, midpoint)))
            rotate.Set(float(np.rad2deg(np.arctan2(direction[1], direction[0]))))
            scale.Set(Gf.Vec3f(float(length), 1., 1.))


class LiveMap:
    """Floating map at the upper right of the Isaac viewport."""
    def __init__(self):
        import omni.ui as ui

        self.provider = ui.ByteImageProvider()
        self.window = ui.Window('LD19 | Live lidar map', width=414, height=296,
                                position_x=530, position_y=78,
                                flags=ui.WINDOW_FLAGS_NO_RESIZE | ui.WINDOW_FLAGS_NO_SCROLLBAR)
        with self.window.frame:
            with ui.VStack(spacing=2):
                self.caption = ui.Label('Forward 180° scan | 226 rays | 10 Hz', height=24)
                ui.ImageWithProvider(self.provider, width=400, height=250)

    def update(self, reading, phase):
        self.caption.text = phase
        image = map_image(reading).resize((400, 250), Image.Resampling.BILINEAR).convert('RGBA')
        self.provider.set_bytes_data(list(image.tobytes()), [400, 250])

    def close(self):
        self.window.visible = False


class LidarRoomDemo:
    def __init__(self, sim, visible=False):
        self.sim = sim
        self.phase = -1
        self.next_scan = 0.
        self.readings = []
        self.start = np.asarray(sim.state()['base_position'])[:2].copy()
        self.fan = ScanFan() if visible else None
        self.map = LiveMap() if visible else None

    @property
    def done(self):
        return self.phase == len(PHASES) - 1 and self.sim.elapsed >= PHASES[-1][0]

    def update(self):
        next_phase = self.phase + 1
        if next_phase < len(PHASES) and self.sim.elapsed >= (0 if self.phase < 0 else PHASES[self.phase][0]):
            self.phase = next_phase
            _, caption, command = PHASES[self.phase]
            self.sim.set_base(*command)
            return caption
        return None

    def post_step(self):
        if self.sim.elapsed + 1e-9 < self.next_scan:
            return
        reading = scan(self.sim)
        self.readings.append(reading)
        self.next_scan = self.sim.elapsed + 1.0 / CONFIG['scan_rate_hz']
        if self.fan:
            self.fan.update(reading)
            self.map.update(reading, PHASES[max(0, self.phase)][1])

    def report(self):
        self.sim.set_base()
        travel = float(np.linalg.norm(np.asarray(self.sim.state()['base_position'])[:2] - self.start))
        center = int(np.argmin(np.abs(ANGLES)))
        center_ranges = [float(r['ranges_m'][center]) for r in self.readings]
        hits = sorted({name for reading in self.readings for name in reading['hit_names'] if name})
        return {'example': 'lidar-room',
                'passed': (travel > 2.0 and len(self.readings) > 30 and len(hits) >= 5
                           and abs(center_ranges[0]-center_ranges[-1]) > 1.0),
                'travel_m': round(travel, 3), 'scan_count': len(self.readings),
                'beam_count': CONFIG['beam_count'], 'distinct_hits': hits,
                'center_range_start_m': round(center_ranges[0], 3),
                'center_range_end_m': round(center_ranges[-1], 3),
                'final_base_xy_m': np.asarray(self.sim.state()['base_position'])[:2].round(3).tolist()}


def run(sim, output):
    demo = LidarRoomDemo(sim)
    while not demo.done:
        demo.update()
        sim.step(50)
        demo.post_step()
    result = demo.report()
    data_path, image_path = save_snapshot(sim, Path(output) / 'lidar_room.json')
    result.update(json=str(data_path), map=str(image_path))
    return result
