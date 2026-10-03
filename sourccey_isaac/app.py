"""Launch the full Sourccey robot in NVIDIA Isaac Sim."""
import argparse
import json
import os
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build', action='store_true', help='Regenerate the USD model and exit')
    p.add_argument('--headless', action='store_true')
    p.add_argument('--seconds', type=float, default=5.)
    p.add_argument('--validate', action='store_true')
    p.add_argument('--validate-cameras', action='store_true',
                   help='Check USD camera poses and fields of view against the source')
    p.add_argument('--smoke-ui', action='store_true', help='Exercise the visible controls and exit')
    p.add_argument('--state', type=Path)
    p.add_argument('--capture', type=Path)
    p.add_argument('--camera-snapshot', type=Path,
                   help='Save a tiled image from all five simulated robot cameras')
    p.add_argument('--lidar-snapshot', type=Path,
                   help='Save a 180-degree scan as JSON plus a 2D PNG map')
    p.add_argument('--example', choices=('camera-tour', 'two-arm-reach', 'workshop-demo', 'lidar-demo', 'lidar-room'),
                   help='Run a camera and motion example (headless unless --view)')
    p.add_argument('--view', action='store_true',
                   help='Play workshop-demo or lidar-room visibly in Isaac')
    p.add_argument('--exit-on-complete', action='store_true',
                   help='Close a visible example when its scripted motion finishes')
    p.add_argument('--example-output', type=Path, default=ROOT/'artifacts/examples',
                   help='Directory for example camera images')
    p.add_argument('--full-elevator-range', action='store_true')
    p.add_argument('--no-traction', action='store_true')
    p.add_argument('--unity-port', type=int)
    p.add_argument('--no-panel', action='store_true')
    args, kit_args = p.parse_known_args()
    if args.example and args.camera_snapshot:
        p.error('--example and --camera-snapshot are separate capture modes')
    if args.example and args.lidar_snapshot:
        p.error('--example and --lidar-snapshot are separate capture modes')
    if args.view and args.example not in ('workshop-demo', 'lidar-room'):
        p.error('--view requires --example workshop-demo or lidar-room')
    if args.view and args.headless:
        p.error('--view and --headless cannot be combined')
    sys.argv = [sys.argv[0]] + kit_args
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': args.headless or args.build or args.validate or args.validate_cameras or bool(args.camera_snapshot) or bool(args.lidar_snapshot) or (bool(args.example) and not args.view),
                         'width': 960, 'height': 640, 'window_width': 1400, 'window_height': 850,
                         'renderer': 'RaytracedLighting', 'anti_aliasing': 0,
                         'multi_gpu': False, 'max_gpu_count': 1, 'fast_shutdown': True,
                         'display_options': 3094}, experience=str(ROOT/'apps/sourccey.kit'))
    receiver = None
    try:
        import carb
        settings = carb.settings.get_settings()
        settings.set('/rtx/translucency/enabled', False)
        settings.set('/rtx/reflections/enabled', False)
        settings.set('/rtx/indirectDiffuse/enabled', False)
        from .model import USD
        if args.build or not USD.exists():
            from .build import build
            build()
        if args.build:
            return
        from isaacsim.core.utils.stage import open_stage
        if not open_stage(str(USD)):
            raise RuntimeError(f'Could not open {USD}')
        for _ in range(5):
            app.update()
        if args.example:
            import omni.usd
            if args.example == 'workshop-demo':
                from examples.scene import add_pickup_scene
                add_pickup_scene(omni.usd.get_context().get_stage())
            elif args.example == 'lidar-demo':
                from examples.scene import add_lidar_scene
                add_lidar_scene(omni.usd.get_context().get_stage())
            elif args.example == 'lidar-room':
                from examples.scene import add_lidar_room
                add_lidar_room(omni.usd.get_context().get_stage())
            else:
                from examples.scene import add_workbench
                add_workbench(omni.usd.get_context().get_stage())
        from .simulation import Simulation, DT
        sim = Simulation(args.full_elevator_range, not args.no_traction,
                         pickup_props=args.example == 'workshop-demo')
        from isaacsim.core.utils.viewports import set_camera_view
        import numpy as np
        if args.example == 'lidar-room':
            set_camera_view(eye=np.array([5.3, -2.4, 5.2]), target=np.array([1.8, 0., .45]))
        else:
            set_camera_view(eye=np.array([1.3, 1.5, 1.15]), target=np.array([0., 0., .65]))
        if args.unity_port:
            from .bridge import PreviewReceiver
            receiver = PreviewReceiver(args.unity_port)
        if args.validate:
            from .validate import validate
            validate(sim, ROOT/'docs/physics_validation.json')
        elif args.validate_cameras:
            from pxr import Usd
            from .camera_validation import validate
            result = validate(Usd.Stage.Open(str(USD)), ROOT/'docs/camera_validation.json')
            print('SOURCCEY_CAMERAS ' + json.dumps(result), flush=True)
        elif args.example and not args.view:
            if args.example == 'camera-tour':
                from examples.camera_tour import run
            elif args.example == 'two-arm-reach':
                from examples.two_arm_reach import run
            elif args.example == 'lidar-demo':
                from examples.lidar_demo import run
            elif args.example == 'lidar-room':
                from examples.lidar_room import run
            else:
                from examples.workshop_demo import run
            result = run(sim, args.example_output / args.example)
            print('SOURCCEY_EXAMPLE ' + json.dumps(result), flush=True)
            if not result['passed']:
                raise RuntimeError(f"Example failed checks: {result}")
        elif args.headless or args.camera_snapshot or args.lidar_snapshot:
            for _ in range(max(0, round(args.seconds/DT))):
                if receiver:
                    receiver.poll(sim)
                sim.step()
            state = sim.state()
            if args.state:
                args.state.parent.mkdir(parents=True, exist_ok=True)
                args.state.write_text(json.dumps(state, indent=2))
            if args.lidar_snapshot:
                from .lidar import save_snapshot
                data_path, image_path = save_snapshot(sim, args.lidar_snapshot)
                print('SOURCCEY_LIDAR_SNAPSHOT '+json.dumps([str(data_path), str(image_path)]), flush=True)
            if args.camera_snapshot:
                from .cameras import save_tiled
                print('SOURCCEY_CAMERA_SNAPSHOT ' + str(save_tiled(sim, args.camera_snapshot)), flush=True)
            print('SOURCCEY_STATE '+json.dumps(state), flush=True)
        else:
            from .panel import Panel
            panel = None if args.no_panel or receiver else Panel(sim)
            if args.view and panel is None:
                raise ValueError('--view requires the Sourccey control panel')
            if args.view:
                if args.example == 'lidar-room':
                    from examples.lidar_room import LidarRoomDemo
                    demo = LidarRoomDemo(sim, visible=True)
                    panel.status.text = 'LD19 room traversal starting; live map at upper right'
                else:
                    from examples.workshop_demo import WorkshopDemo
                    demo = WorkshopDemo(sim)
                    panel.status.text = 'Workshop demo starting; select a camera at any time'
            else:
                demo = None
            if args.smoke_ui:
                if panel is None:
                    raise ValueError('--smoke-ui requires the panel')
                for _ in range(10):
                    sim.world.render()
                panel.joint_models['left_elbow_flex'].set_value(-110.)
                assert abs(sim.target_positions['left_elbow_flex']-np.deg2rad(-110)) < 1e-5
                panel.base_models[0].set_value(.2)
                assert abs(sim.base_command[0]-.2) < 1e-5
                panel.stop()
                assert np.all(sim.base_command == 0)
                panel.reset()
                from .camera_config import CAMERA_PATHS
                from omni.kit.viewport.utility import get_active_viewport
                panel.select_camera('front_left')
                assert str(get_active_viewport().camera_path) == CAMERA_PATHS['front_left']
                panel.select_camera(None)
                panel.save_lidar()
                lidar_output = ROOT/'artifacts/lidar_scan.json'
                from .lidar import CONFIG as LIDAR_CONFIG
                assert len(json.loads(lidar_output.read_text(encoding='utf-8'))['ranges_m']) == LIDAR_CONFIG['beam_count']
                assert lidar_output.with_suffix('.png').exists()
                sim.step(250)
                for name, value in sim.defaults.items():
                    if name not in ('front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel'):
                        assert abs(sim.positions()[name]-value) < .01, name
                result = {'passed': True, 'panel_created': True, 'camera_selector': True,
                          'lidar_snapshot': True,
                          'joint_callback': True,
                          'base_callback': True, 'stop': True, 'reset': True, 'state': sim.state()}
                (ROOT/'docs/ui_validation.json').write_text(json.dumps(result, indent=2)+'\n')
                print('SOURCCEY_UI_CHECK '+json.dumps(result), flush=True)
                panel.closed = True
            last, accumulator = time.perf_counter(), 0.
            paused_last = False
            lidar_reported = False
            print('SOURCCEY_READY '+json.dumps(sim.state()), flush=True)
            while app.is_running() and (panel is None or not panel.closed):
                now = time.perf_counter()
                accumulator += min(now-last, .05)
                last = now
                if receiver:
                    receiver.poll(sim)
                if panel:
                    panel.update()
                    if demo is not None and paused_last and not panel.paused:
                        demo.resume()
                    paused_last = panel.paused
                while accumulator >= DT:
                    if sim.world.is_playing() and (panel is None or not panel.paused):
                        if args.example == 'workshop-demo' and demo is not None and demo.done and sim.elapsed >= 17.6:
                            sim.reset()
                            demo.reset_props()
                            demo = WorkshopDemo(sim)
                            panel.status.text = 'Replaying workshop demo'
                            print('SOURCCEY_DEMO_REPLAY', flush=True)
                        if demo is not None and not demo.done:
                            caption = demo.update()
                            if caption:
                                panel.status.text = caption
                                print('SOURCCEY_DEMO_PHASE ' + caption, flush=True)
                        sim.step()
                        if demo is not None:
                            demo.post_step()
                    accumulator -= DT
                if args.example == 'lidar-room' and demo is not None and demo.done and not lidar_reported:
                    result = demo.report()
                    print('SOURCCEY_EXAMPLE ' + json.dumps(result), flush=True)
                    if not result['passed']:
                        raise RuntimeError(f'Lidar room example failed checks: {result}')
                    panel.status.text = 'LD19 room traversal complete; close Isaac when finished'
                    lidar_reported = True
                    if args.exit_on_complete:
                        panel.closed = True
                sim.world.render()
                time.sleep(.001)
        if args.capture:
            from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
            args.capture.parent.mkdir(parents=True, exist_ok=True)
            for _ in range(10):
                sim.world.render()
            capture = capture_viewport_to_file(get_active_viewport(), str(args.capture))
            for _ in range(120):
                app.update()
                if args.capture.exists():
                    break
            if not args.capture.exists():
                raise RuntimeError(f'Viewport capture did not finish: {args.capture}')
    except Exception:
        (ROOT/'logs').mkdir(exist_ok=True)
        detail = traceback.format_exc()
        (ROOT/'logs/error.log').write_text(detail, encoding='utf-8')
        print(detail, file=sys.stderr, flush=True)
        # Kit's fast shutdown terminates with zero before Python can re-raise.
        # On a fatal application error, preserve a failure status for the launcher.
        sys.stdout.flush()
        os._exit(1)
    finally:
        if receiver:
            receiver.close()
        app.close()
