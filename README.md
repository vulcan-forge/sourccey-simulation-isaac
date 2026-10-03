# Sourccey robot in NVIDIA Isaac Sim

The configured Sourccey robot from the sibling MuJoCo project, converted to an
Isaac Sim USD articulation with PhysX dynamics. All 17 joints, CAD axes, visual
meshes, masses, joint limits, and startup targets are carried across.

Licensed under the [MIT License](LICENSE). Copyright (c) 2026 Vulcan Robotics.

![Sourccey rendered in Isaac Sim](docs/preview.png)

## Launch

On a fresh Windows checkout, install Python 3.10 or newer and run `setup.cmd` once. It downloads the official Isaac Sim runtime, builds the launcher, and generates the scene.

After setup, double-click **SourcceyIsaac.exe**. It opens Isaac Sim and the Sourccey controls.
The first launch can take several minutes while NVIDIA compiles shaders.
Startup output is written to `logs/latest.log`; Python errors also go to
`logs/error.log`. The launcher prevents duplicate launches through the `.exe`.
Use **Quit Sourccey** to close the application completely.

Keep the `.exe` in this folder. The multi-gigabyte `.runtime` is ignored by Git, so the launcher alone cannot start on a fresh clone. Unlike the small MuJoCo executable, Isaac needs
its multi-gigabyte `.runtime` directory and the project's models and scripts.
Copy the whole folder when moving this installation to another Windows PC.

Commands from this folder:

```bat
run.cmd
run.cmd --headless --seconds 5 --state artifacts\state.json
run.cmd --validate
run.cmd --build
run.cmd --unity-port 8765
run.cmd --camera-snapshot artifacts\camera_feeds.png --seconds 0.1
run.cmd --example camera-tour
run.cmd --example two-arm-reach
run.cmd --example workshop-demo --view
run.cmd --example lidar-demo
run.cmd --example lidar-room --view
run.cmd --validate-cameras
```

`--no-traction` disables the equivalent mecanum forces. `--full-elevator-range`
extends the bottom elevator endpoint from -0.3104 m to -0.315 m; the upper
endpoint remains -0.0142 m. `--no-panel` opens only the Isaac interface.

## Simulated cameras and examples

The generated USD contains `front_left`, `front_right`, `bottom`, `wrist_left`,
and `wrist_right` camera prims under the moving robot bodies. The control panel
has an expanded **Simulated cameras** section: click a named camera to switch
the live Isaac viewport, or **Overview** to return. The viewport's native
camera menu can select these prims too. On this 8 GB GPU, the app renders one
live viewport at a time rather than five simultaneous ray-traced viewports.

`--camera-snapshot` saves a labeled 320 x 240 image from each camera in one
contact sheet. The two headless examples run scripted PhysX motion, then save
the same five-view sheet under `artifacts/examples/`. Both place a solid
workbench and two colored blocks ahead of the robot as camera scenery;
`camera-tour` drives forward and tilts both wrists, while `two-arm-reach` uses
IK to move both arms to nearby reachable targets and reports measured reach
error. The examples capture after motion because Isaac's standalone Replicator
capture step resets the
live articulation handle. Neither is a validated object-grasping task.

For a **live pick-and-place simulation**, run `run.cmd --example workshop-demo
--view`. The robot rolls to a table, opens both hands, reaches down to the red
and blue parcels, closes its grippers, lifts both parcels, moves them outward,
and releases them onto colored pads. The control panel shows each phase; choose
Overview or any of the five simulated cameras while it runs. Pause / resume
works during playback, and the sequence repeats automatically. The parcels
have collision and dynamic rigid-body physics when free. The carry uses an
**assisted kinematic attachment** after the hands approach the parcels, then
returns them to PhysX when released. This is a visible manipulation demo, not
proof of a contact-only finger grasp. Run `run.cmd --example
workshop-demo` without `--view` to run the same sequence headlessly and save
`artifacts/examples/workshop-demo/workshop_demo.png`.

The camera source [camera_poses.json](models/source/camera_poses.json) is the
same provisional MuJoCo pose set. `run.cmd --validate-cameras` verifies that
the USD body-local mounts and fields of view match the source. These are
simulated pinhole views, not independently calibrated physical feeds. Some views are
partly occluded by the CAD shell at the default pose, especially the Isaac
front-left feed; the bottom mounting angle is still a manual estimate.

## Forward planar lidar

The lower front slit has a simulated horizontal slice for the **FHL-LD19**.
The physical scanner rotates through 360 degrees, but this model exposes only
the forward 180 degrees through the robot's slit; that aperture is provisional.
It uses 226 rays at 0.8-degree spacing, a 0.02 m minimum and a 12 m maximum,
with the LD19's typical 10 Hz scan period recorded in each scan.
In the control panel, **Save lidar scan and XY map** writes
`artifacts/lidar_scan.json` and `artifacts/lidar_scan.png`. The PNG shows a
robot-relative top-down slice, with the sensor at the orange dot and returns
in green. Scripts can call `sourccey_isaac.lidar.scan(sim)`; `inf` means no
return. The PhysX query filters the robot's own collision shapes.

To check three known obstacles and view the result:

```powershell
.\run.cmd --example lidar-demo
Invoke-Item .\artifacts\examples\lidar-demo\lidar_demo.png
```

`run.cmd --lidar-snapshot artifacts\lidar_scan.json --seconds 0.1` scans the
ordinary robot scene. With no obstacles at the scan height, its ranges are
empty. The optical point now sits inside the front-panel slit: panel-local
height 0.205 m, robot-forward position 0.17991 m, about 27 mm behind the outer
front surface. The CAD slice exposes 180.00 degrees at 0.01-degree probe
resolution there; 1 mm
inward gives about 178.8 degrees, and 1 mm outward about 181.2 degrees.
The simulated output is explicitly limited to -90 through +90 degrees.
The exact real optical center is absent from the URDF;
the CAD-derived mount and scan assumptions are documented in
[lidar_config.json](models/source/lidar_config.json). Range, angular resolution,
and scan rate follow the [FHL-LD19 manufacturer's specifications](https://wiki.youyeetoo.com/en/Lidar/D300).
The ordinary panel takes an on-demand geometric scan. Reflectivity, hardware
noise, and individual rotating beam timing are not modeled.

For a live room traversal like the MuJoCo example, run
`run.cmd --example lidar-room --view`. Sourccey drives past six solid objects,
slides through the aisle, and turns toward the far wall. A floating LD19 map
appears at the upper right of the viewport; teal render-only rays show the
first 2.2 m of the scan from the lower-front mount. The physics query still
returns ranges up to 12 m. The map refreshes at the nominal 10 Hz while this
example runs. Use `run.cmd --example lidar-room` for a headless measured run
that saves `artifacts/examples/lidar-room/lidar_room.json` and its PNG map.

## Controls and defaults

The panel has forward, strafe, and yaw sliders; release a base slider to stop.
**STOP BASE**, **Reset pose**, **Pause / resume**, joint sliders, grippers, and
per-arm XYZ reach targets match the MuJoCo application's controls. Joint
sliders display degrees. The following source targets are radians, except the
linear actuator, which is in meters:

| Joint | Startup target |
| --- | ---: |
| Linear actuator | -0.0142 m |
| Left / right shoulder pan | -0.785 / -0.801 rad |
| Left / right shoulder lift | +2.01 / -2.01 rad |
| Both elbow flexes | -2.0 rad |
| Left / right wrist flex | +0.691 / -0.723 rad |
| Both wrist rolls | 0 rad |
| Both grippers | -0.087266463 rad (-5 degrees, closed) |
| All four wheels | 0 rad/s |

Both elbow lower stops are -135 degrees. Joint limits use the same overrides
as MuJoCo. The elevator upper joint stop, target cap, and initial position are
all -0.0142 m. Reset restores the complete pose and stops the wheels.

XYZ targets are meters from the moving shoulder mount, with X forward, Y left,
and Z up. IK controls shoulder pan, shoulder lift, and elbow flex. Wrist flex
and roll remain independent. Freeze/resume preserves the last reach target
and discards slider changes made while frozen.

The optional receive-only Unity bridge accepts the existing
`sourccey.mujoco.v1` packet format so the existing Unity sender continues to work.
Use the same port in both applications. Only one simulator may bind that port
at a time. After 300 ms without valid input, the base stops and the arms hold
their last targets. No code here connects to the physical robot.
The Unity headset-to-Isaac workflow has not been exercised here.

## Runtime and hardware

This installation pins **Isaac Sim 5.1.0**. It supports the existing NVIDIA
591.86 driver; the current 6.1 release specifies a newer driver. 5.1 is an older,
now unsupported NVIDIA release. No system drivers are changed by this project.

This PC's RTX 2070 SUPER has 8 GB VRAM, below NVIDIA's published 16 GB minimum
for 5.1. The project uses a single robot, CPU PhysX, a small viewport, and reduced
rendering effects. This is a practical compatibility attempt, not a claim that
the PC meets NVIDIA's support requirements. Close other GPU-heavy applications
before launching. GPU memory exhaustion or shader/driver issues can still stop
Isaac even when the model and controls are correct.
On this workstation, the PhysX checks and the visible control-panel/render
check both passed at the configured 960 x 640 viewport size.

On a fresh checkout, install Python 3.10 or newer and run `setup.cmd`. It downloads
the official Windows runtime into `.downloads`, extracts it into `.runtime`,
compiles the small Windows launcher, and generates `models/sourccey.usdc`.
The archive is about 7.9 GB; allow at least 50 GB free during setup. Running the
NVIDIA runtime accepts its license through `OMNI_KIT_ACCEPT_EULA=YES` in
`run.cmd`; see [NVIDIA's Isaac Sim license](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/common/licenses.html).
`package.cmd` rebuilds only the launcher; Python/model changes do not need
repackaging. Use `run.cmd --build` after changing the reference model.

## Model, implementation, and verification

- `models/source/`: preserved URDF, 171 STL meshes, and source provenance.
- `models/reference/sourccey.xml`: configured MuJoCo snapshot, including the
  accepted pose and limit overrides. It is conversion input, not a runtime
  dependency on the MuJoCo Python package.
- `sourccey_isaac/model.py`: model parsing, fixed-part mass/inertia merging,
  forward kinematics, and arm IK.
- `sourccey_isaac/build.py`: reproducible USD/PhysX scene generation.
- `sourccey_isaac/simulation.py`: Isaac articulation drives and wheel traction.
- `sourccey_isaac/panel.py`: native Isaac UI.
- `docs/kinematic_parity.json`: comparison against the actual MuJoCo model at
  startup and 21 random poses. All 132 CAD frames match, with 17 joints and
  total mass 169.041941 kg preserved. Fixed links merge into 18 moving bodies.
- `docs/physics_validation.json`: created by `run.cmd --validate`, including
  actual joint tracking, elevator travel, forward/strafe/yaw, stopping, and IK.
  Only a report with `passed: true` establishes that those PhysX checks passed.
- `docs/ui_validation.json`: the visible panel was created; joint and base
  callbacks, stopping, and restoring the measured startup pose all passed.

To repeat the independent MuJoCo comparison using the sibling project's Python:

```bat
..\SourcceyMuJoCo\.venv\Scripts\python.exe tools\compare_mujoco.py
```

PhysX and MuJoCo have different solvers, so matching targets and kinematics do
not imply identical dynamic trajectories. Wheels retain the contact-based,
load-limited equivalent mecanum model on a flat, stationary floor; individual
rollers are not modeled. Robot self-collision is disabled because the CAD has
overlapping mating parts. Arm/environment collision uses convex part hulls;
the chassis uses a box and each wheel uses a sphere. These are simulation
approximations, not a measured hardware dynamics model or grasp planner.

Arm drives use idealized gravity feed-forward and finite drive/effort caps;
PhysX applies drive and feed-forward forces through separate APIs. Their
combined saturation differs from the MuJoCo implementation. The exported USD
has drives and joint limits, but **wheel traction, live controls, and gravity
compensation require running this application**.

References: [Isaac Sim 5.1 installation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_workstation.html),
[system requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html),
[articulation and rigid-body APIs](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.core.prims/docs/index.html).
