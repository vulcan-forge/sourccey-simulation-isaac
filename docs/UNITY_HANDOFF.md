# Sourccey robot: MuJoCo integration handoff

This repository is for an Isaac Sim simulation of the Sourccey robot. The source robot model and the existing Unity teleoperation implementation are in:

[Sourccey-VR-Controller](https://github.com/ChristopherMaselli/Sourccey-VR-Controller)

Read that repository before building a second robot model. Keep the MuJoCo model and code in this repository; treat the Unity repository as the source of geometry, joint names, and established control behavior. The current Unity scene organization is being changed, so use the URDF and source files listed below rather than depending on scene names.

## Start with these files

| Purpose | Path relative to the Unity repository |
| --- | --- |
| Full robot URDF | `SourcceyURDF/SourcceyMkV.urdf` |
| STL geometry referenced by the URDF | `SourcceyURDF/meshes/` |
| URDF design notes and known physics limitations | `SourcceyURDF/README.md` |
| Full URDF generator and checks | `UrdfStaging/convert_full.py`, `UrdfStaging/validate_full.py` |
| Optional portable URDF ZIP generator | `UrdfStaging/package_full.py` |
| Unity full-body preview and wheel mixing | `Assets/Scripts/Controllers/FullRobotVisualDriver.cs` |
| VR end-effector mapping, freeze/rebase, wrist controls | `Assets/Scripts/IKSolver/VRToRobotMapper2.cs` |
| Unity arm IK solver | `Assets/Scripts/IKSolver/IKRobotCustom.cs` |
| Joint-angle sampling and teleop output | `Assets/Scripts/UnityMotorBridge/UnityJointAngleProvider.cs`, `UnityArmTeleopProcessor.cs` |
| Angle/servo/action conversion | `Assets/Scripts/UnityMotorBridge/UnityIKMotorBridge.cs` |
| Robot command structure and dispatch | `Assets/Scripts/Data/Models/RobotAction.cs`, `Assets/Scripts/Controllers/RobotController.cs` |
| Physical robot transport, only if needed | `Assets/Scripts/Communication/ZMQ/ZMQClient.cs` |
| Standalone arm reference URDFs | `Assets/Urdf/ArmLeft/ArmLeft.urdf`, `Assets/Urdf/ArmRight/ArmRight.urdf` |

Use the full robot URDF for **both arms**. Its arm geometry and joints come from the full Fusion assembly, with the working standalone arms' joint roles and limits applied by the conversion script. Do not substitute the standalone arm meshes into the full robot or use the Unity prefab as the canonical geometry. The Unity full-body preview still uses standalone articulated rigs to calculate arm poses, then copies the resulting angles onto the full assembly arm visuals; this is an implementation detail to port, not a different robot model.

## Model inventory and coordinates

The checked-in URDF has `base_link`, 132 links, 131 joints (17 movable), and 171 distinct referenced STL files. All mesh paths are relative to `SourcceyURDF/SourcceyMkV.urdf`; copy the whole `SourcceyURDF` folder, preserving `meshes/`. The STL vertices are in millimeters and the URDF mesh entries apply `scale="0.001 0.001 0.001"`. Joint translations are in meters, revolute limits in radians, and the CAD joint axes are already in the URDF. Validate scale and axis directions visually after import.

Movable joints:

- Left arm: `left_shoulder_pan`, `left_shoulder_lift`, `left_elbow_flex`, `left_wrist_flex`, `left_wrist_roll`, `left_gripper`.
- Right arm: the same six names with `right_` prefixes.
- Wheels: `front_left_wheel`, `front_right_wheel`, `rear_left_wheel`, `rear_right_wheel` (continuous).
- Shoulder elevator: `linear_actuator` (prismatic); the two shoulder mounts descend from this link, so both arms travel together.

The URDF's `linear_actuator` range is **-0.315 to 0 m**, with the captured CAD pose at zero. Left shoulder lift is -120 to +120 degrees; right shoulder lift is -120 to +110 degrees. Both grippers range from -5 degrees (closed with clearance) to +60 degrees (open). Respect the URDF limits for every joint and verify the physical closure gap. The gripper is a single revolute joint per side; the opposite finger is fixed in the CAD model.

The Unity preview intentionally uses a narrower elevator travel: `actuatorStrokeMeters = 0.285` and `actuatorVerticalOffsetMeters = -0.0254` in `FullRobotVisualDriver.cs`. Its command value `z_vel` ranges from -100 to +100, with +100 at the top; it maps to approximately -0.0254 through -0.3104 m in that preview. The name `z_vel` is historical: the Unity preview treats it as an **actuator position request**, not a physical velocity. Decide whether MuJoCo should follow this demonstrated preview range or use the full URDF joint range, and test both end stops before connecting controls.

## Control behavior to reproduce

1. **Arm targets and IK:** `VRToRobotMapper2.cs` maps controller/hand motion to an end-effector target and handles freeze, unfreeze, robot-relative motion, and wrist flex/roll. `IKRobotCustom.cs` solves the three primary arm joints with Unity `ArticulationBody` drives. Port the behavior or write a MuJoCo-compatible IK solver against the full URDF chain; Unity `ArticulationBody`, `Transform`, and `MonoBehaviour` classes cannot run in MuJoCo. Keep frozen targets in the robot frame so they travel and turn with the chassis. Rebase hand/controller input when unfreezing after chassis motion so a forward hand movement remains forward relative to Sourccey.
2. **Joint and hardware mapping:** `UnityJointAngleProvider.cs` reads the virtual arm joints; `UnityArmTeleopProcessor.cs` and `UnityIKMotorBridge.cs` convert angles to the robot's action fields and optional servo ticks. Left/right signs and offsets differ. Do not equate real-robot action numbers directly with URDF joint radians. Inspect the sign maps, offsets, gripper inversion, and calibration code before reusing the command protocol. The calibration JSON files beside the bridge are hardware references, not MuJoCo physics parameters.
3. **Base movement and wheels:** `RobotAction` fields `x_vel`, `y_vel`, and `theta_vel` express normalized forward, lateral, and yaw commands. `FullRobotVisualDriver.cs` currently caps these at **0.6 m/s** translation and **90 deg/s** yaw. Its approximate mecanum mixing uses wheel radius **0.052 m** and half-length-plus-half-width **0.30 m**. With forward speed `v`, left speed `s`, yaw rate `w` in rad/s, and `L=0.30`, it calculates `FL=(v-s-Lw)/r`, `FR=(v+s+Lw)/r`, `RL=(v+s-Lw)/r`, `RR=(v-s+Lw)/r`. It negates **right wheel** rates when writing to the CAD joints because their axes point opposite the left wheels. Confirm all four signs in MuJoCo with forward, strafe, and turn tests. The Unity preview moves the chassis transform directly and merely spins the wheels visually; it does not demonstrate physical mecanum traction.
4. **Actuator and grippers:** The full-body driver copies controlled arm angles onto the Mk.V visual joints. It also maps a 0–100 gripper closure fraction to **+60 to -5 degrees** on each full-assembly gripper. The physical robot action uses opposite left/right gripper percent conventions; see `UnityArmTeleopProcessor.cs` and `RobotController.cs`. In simulation, command the URDF gripper angles, respecting the -5-degree closure stop; do not send servo percentages straight to MuJoCo joints.
5. **Real robot interface:** `RobotController.cs` assembles/sends `RobotAction`; `ZMQClient.cs` is the actual transport. The new simulation should be usable without a robot connection. If physical-robot interoperability is desired later, build it as an optional adapter and preserve the freeze-on-connect behavior in `RobotController.cs` so a reconnect cannot immediately move a real arm.

## MuJoCo implementation order

1. Copy `SourcceyURDF/SourcceyMkV.urdf` and its entire `meshes/` directory into this repository under a clear model folder. Keep a note of the source Git commit and do not edit the source URDF in place. Alternatively, run `python UrdfStaging/package_full.py` in the Unity repository to create the portable ZIP under its ignored `Deliverables/` folder.
2. Load the copied URDF with the installed MuJoCo Python package and report any compiler or mesh errors. A first smoke check can use `mujoco.MjModel.from_xml_path(path_to_urdf)` followed by `mujoco.MjData(model)`. Keep the relative mesh layout intact. Inspect the compiled joint names and types; the imported model may optimize fixed links.
3. Create a separate MuJoCo/MJCF working model as needed. Add a **free base**, ground plane, gravity, actuators, and contact geometry if the goal is a mobile physical robot; the URDF alone describes joints and meshes but not a complete driving/control system. Preserve a reproducible conversion from the copied URDF. Start with simple collision primitives or convex approximations for chassis and wheels while keeping the detailed STLs for visuals. The source currently uses many detailed STL meshes for both visual and collision geometry.
4. Add position-controlled arm and gripper joints, a position-controlled shoulder elevator, and independently driven wheel joints. Tune force limits, damping, friction, timestep, and contact based on measurements; the URDF's very large arm effort/velocity numbers should not be treated as validated motor ratings.
5. Implement and test the kinematic controller separately from physics: command each named joint at its midpoint and limits; check the left/right arm directions, shoulder mounts, gripper clearance, elevator travel, and all wheel spin signs. Then add robot-relative end-effector IK and the freeze/rebase behavior. Finally enable contact-based driving and test forward, lateral, and yaw motion independently.

**Physics caveats:** Fusion supplied masses and inertias before detached duplicate meshes were filtered; the URDF totals about **169.042 kg**, and `SourcceyURDF/README.md` explicitly calls for recapturing per-body mass properties and checking CAD materials before trusting dynamics quantitatively. The URDF wheel meshes show mecanum wheels, but four spinning wheel joints alone will not reproduce lateral mecanum motion. Model rollers or a validated equivalent contact/friction scheme and check lateral traction. These are simulation tasks, not evidence that the URDF joint tree is missing.

## Acceptance checks

- The copied URDF loads from this repository with **no missing mesh files** and an intelligible visual scale.
- The model exposes all **17 movable joints** above; both arms remain attached to the `linear_actuator` descendant chain.
- Both arms can reach a comparable neutral/raptor pose within their respective shoulder limits; both grippers close with visible clearance and open in the correct direction.
- Elevator top/bottom are verified against the selected travel range, with the shoulder mounts and both arms moving together.
- Separate forward, strafe, and yaw commands produce the intended chassis motion and wheel spin directions. Clearly label any kinematic-only prototype as such until physical wheel contact is implemented.
- Freezing arm targets, moving/turning the base, then unfreezing does not leave targets behind or rotate hand translation into the wrong robot direction.
- With no physical robot connection, MuJoCo simulation and controls still operate.

## MuJoCo references

- [Official modeling guide: URDF import and extensions](https://mujoco.readthedocs.io/en/stable/modeling.html#urdf-extensions)
- [Official Python API: model loading](https://mujoco.readthedocs.io/en/stable/python.html)
- [Official XML reference: compiler and collision settings](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [Official contact and friction model](https://mujoco.readthedocs.io/en/stable/computation/index.html#contact)
