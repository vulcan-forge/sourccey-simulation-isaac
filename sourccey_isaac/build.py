"""Generate a self-contained USD articulation for Isaac Sim from the saved model."""
import json
import hashlib
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdLux, PhysxSchema, Sdf
from .model import ROOT, SOURCE, USD, RobotModel, WHEELS, mesh_points, vector
from .camera_config import CAMERA_NAMES, CAMERA_PATHS, CAMERA_POSES, CAMERA_SOURCE


def quat(matrix):
    q = Rotation.from_matrix(matrix).as_quat()
    return Gf.Quatf(float(q[3]), Gf.Vec3f(*map(float, q[:3])))


def pose(prim, matrix):
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, matrix[:3, 3])))
    xf.AddOrientOp().Set(quat(matrix[:3, :3]))


def axis_frame(axis):
    # Construct a right-handed frame whose X axis is the CAD joint axis.
    helper = np.array([0., 0., 1.]) if abs(axis[2]) < .9 else np.array([0., 1., 0.])
    y = np.cross(helper, axis); y /= np.linalg.norm(y)
    return np.column_stack((axis, y, np.cross(axis, y)))


def build():
    model = RobotModel()
    USD.parent.mkdir(parents=True, exist_ok=True)
    if USD.exists():
        # Reuse and clear an existing layer so rebuild works in the same Kit session.
        layer = Sdf.Layer.FindOrOpen(str(USD)); layer.Clear()
        stage = Usd.Stage.Open(layer)
    else:
        stage = Usd.Stage.CreateNew(str(USD))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(world.GetPrim())
    scene = UsdPhysics.Scene.Define(stage, '/World/physicsScene')
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1)); scene.CreateGravityMagnitudeAttr(9.81)
    ps = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    ps.CreateEnableGPUDynamicsAttr(False)
    ps.CreateBroadphaseTypeAttr('MBP')
    ps.CreateSolverTypeAttr('TGS')
    ps.CreateTimeStepsPerSecondAttr(500)
    ground = UsdGeom.Cube.Define(stage, '/World/Ground')
    ground.CreateSizeAttr(1.0)
    ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -.05))
    ground.AddScaleOp().Set(Gf.Vec3f(200, 200, .1))
    ground.CreateDisplayColorAttr([Gf.Vec3f(.19, .22, .27)])
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    mat = UsdPhysics.MaterialAPI.Apply(stage.DefinePrim('/World/PhysicsMaterial', 'Material'))
    mat.CreateStaticFrictionAttr(0); mat.CreateDynamicFrictionAttr(0); mat.CreateRestitutionAttr(0)
    PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr('max')
    from pxr import UsdShade
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(
        UsdShade.Material(mat.GetPrim()), UsdShade.Tokens.weakerThanDescendants, 'physics')
    UsdGeom.Xform.Define(stage, '/World/Sourccey')
    frames = model.frames(model.defaults)
    cache = {}
    for name, group in model.groups.items():
        path = '/World/Sourccey/'+name
        body = UsdGeom.Xform.Define(stage, path)
        pose(body.GetPrim(), frames[name])
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(body.GetPrim())
        rb.CreateLinearDampingAttr(0.0); rb.CreateAngularDampingAttr(0.0)
        rb.CreateMaxDepenetrationVelocityAttr(2.0)
        mass = UsdPhysics.MassAPI.Apply(body.GetPrim())
        mass.CreateMassAttr(group['mass'])
        mass.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, group['com'])))
        eigenvalues, eigenvectors = np.linalg.eigh(group['inertia'])
        if np.linalg.det(eigenvectors) < 0:
            eigenvectors[:, 0] *= -1
        assert np.all(eigenvalues > 0), name
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, eigenvalues)))
        mass.CreatePrincipalAxesAttr(quat(eigenvectors))
        for geom, matrix in group['geoms']:
            gp = path+'/'+geom.get('name')
            kind = geom.get('type')
            collision = geom.get('group') == '3'
            if kind == 'mesh':
                mesh = UsdGeom.Mesh.Define(stage, gp)
                key = geom.get('mesh')
                if key not in cache:
                    cache[key] = mesh_points(model.meshes[key]).astype(np.float32)
                points = cache[key]
                mesh.CreatePointsAttr(points)
                mesh.CreateFaceVertexCountsAttr(np.full(len(points)//3, 3, dtype=np.int32))
                mesh.CreateFaceVertexIndicesAttr(np.arange(len(points), dtype=np.int32))
                mesh.CreateSubdivisionSchemeAttr('none')
                mesh.CreateDoubleSidedAttr(True)
                g = mesh
                if collision:
                    UsdPhysics.MeshCollisionAPI.Apply(g.GetPrim()).CreateApproximationAttr('convexHull')
            elif kind == 'sphere':
                g = UsdGeom.Sphere.Define(stage, gp)
                g.CreateRadiusAttr(float(geom.get('size')))
            elif kind == 'box':
                g = UsdGeom.Cube.Define(stage, gp)
                g.CreateSizeAttr(1.0)
            else:
                raise ValueError(f'Unsupported geometry {kind}')
            pose(g.GetPrim(), matrix)
            if kind == 'box':
                g.AddScaleOp().Set(Gf.Vec3f(*map(float, vector(geom.get('size'))*2)))
            if collision:
                UsdPhysics.CollisionAPI.Apply(g.GetPrim())
                ca = PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
                ca.CreateContactOffsetAttr(.002); ca.CreateRestOffsetAttr(0.0)
                g.CreateVisibilityAttr('invisible')
                friction = float(vector(geom.get('friction'), '1 .005 .0001')[0])
                contact_material = mat
                if friction:
                    contact_material = UsdPhysics.MaterialAPI.Apply(stage.DefinePrim(
                        '/World/ContactMaterial_'+str(friction).replace('.', '_'), 'Material'))
                    contact_material.CreateStaticFrictionAttr(friction)
                    contact_material.CreateDynamicFrictionAttr(friction)
                    contact_material.CreateRestitutionAttr(0.)
                    PhysxSchema.PhysxMaterialAPI.Apply(contact_material.GetPrim()).CreateFrictionCombineModeAttr('max')
                UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(
                    UsdShade.Material(contact_material.GetPrim()), UsdShade.Tokens.weakerThanDescendants, 'physics')
            else:
                g.CreateDisplayColorAttr([Gf.Vec3f(*map(float, vector(geom.get('rgba'))[:3]))])
    # Keep the fitted optical poses in their original CAD-local body frames.
    # The base camera source calls base_link "robot_root"; fixed descendants
    # are transformed into their merged Isaac rigid body's frame.
    for name in CAMERA_NAMES:
        source = CAMERA_POSES[name]
        body_name = 'base_link' if source['body'] == 'robot_root' else source['body']
        group_name, body_in_group = model.original_frames[body_name]
        camera_in_body = np.eye(4)
        camera_in_body[:3, 3] = source['pos']
        w, x, y, z = source['quat_wxyz']
        camera_in_body[:3, :3] = Rotation.from_quat([x, y, z, w]).as_matrix()
        camera_path = CAMERA_PATHS[name]
        assert camera_path.startswith(f'/World/Sourccey/{group_name}/')
        camera = UsdGeom.Camera.Define(stage, camera_path)
        pose(camera.GetPrim(), body_in_group @ camera_in_body)
        fovy = np.deg2rad(source['fovy_deg'])
        vertical_aperture = 24.0
        camera.CreateVerticalApertureAttr(vertical_aperture)
        camera.CreateHorizontalApertureAttr(vertical_aperture * 4 / 3)
        camera.CreateFocalLengthAttr(float(vertical_aperture / (2 * np.tan(fovy / 2))))
        camera.CreateClippingRangeAttr(Gf.Vec2f(.005, 100.0))
    root = stage.GetPrimAtPath('/World/Sourccey/base_link')
    UsdPhysics.ArticulationRootAPI.Apply(root)
    art = PhysxSchema.PhysxArticulationAPI.Apply(root)
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(8)
    UsdGeom.Scope.Define(stage, '/World/Sourccey/Joints')
    for name, j in model.joints.items():
        path = '/World/Sourccey/Joints/'+name
        slide = j['type'] == 'slide'
        joint = (UsdPhysics.PrismaticJoint if slide else UsdPhysics.RevoluteJoint).Define(stage, path)
        joint.CreateBody0Rel().SetTargets(['/World/Sourccey/'+j['parent']])
        joint.CreateBody1Rel().SetTargets(['/World/Sourccey/'+j['child']])
        align = axis_frame(j['axis'])
        joint.CreateAxisAttr('X')
        joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float, j['origin'][:3, 3])))
        joint.CreateLocalRot0Attr(quat(j['origin'][:3, :3] @ align))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0)); joint.CreateLocalRot1Attr(quat(align))
        joint.CreateCollisionEnabledAttr(False)
        scale = 1.0 if slide else 180/np.pi
        if name not in WHEELS:
            joint.CreateLowerLimitAttr(float(j['limits'][0]*scale))
            joint.CreateUpperLimitAttr(float(j['limits'][1]*scale))
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), 'linear' if slide else 'angular')
        drive.CreateTypeAttr('force')
        source = model.actuators[name]
        # USD angular drives are per degree; the high-level Isaac APIs use radians.
        drive.CreateStiffnessAttr(float(source.get('kp', '0'))/scale)
        drive.CreateDampingAttr(float(source.get('kv', '0'))/scale)
        drive.CreateMaxForceAttr(float(vector(source.get('forcerange'))[1]))
        drive.CreateTargetPositionAttr(float(model.defaults[name]*scale) if name not in WHEELS else 0.0)
        drive.CreateTargetVelocityAttr(0.0)
        pj = PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim())
        pj.CreateArmatureAttr(.01 if name in WHEELS else .04)
        pj.CreateJointFrictionAttr(0.0)
    light = UsdLux.DomeLight.Define(stage, '/World/Light')
    light.CreateIntensityAttr(650)
    stage.GetRootLayer().Save()
    metadata = {'source_sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
                'camera_source_sha256': hashlib.sha256(CAMERA_SOURCE.read_bytes()).hexdigest(),
                'cameras': list(CAMERA_NAMES),
                'joints': list(model.joints), 'joint_count': len(model.joints),
                'rigid_bodies': len(model.groups), 'mass_kg': sum(g['mass'] for g in model.groups.values()),
                'startup_pose_si': model.defaults, 'usd': str(USD.relative_to(ROOT)),
                'physics': 'PhysX CPU, floating-base articulation, equivalent mecanum contact forces'}
    (ROOT/'models/build_info.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print(json.dumps(metadata, indent=2), flush=True)
    return USD


if __name__ == '__main__':
    build()
