"""A compact, solid workbench used as visual context for camera examples."""
from pxr import Gf, UsdGeom, UsdPhysics


def add_workbench(stage):
    UsdGeom.Xform.Define(stage, '/World/CameraExample')

    def box(name, position, scale, color):
        cube = UsdGeom.Cube.Define(stage, '/World/CameraExample/' + name)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*position))
        cube.AddScaleOp().Set(Gf.Vec3f(*scale))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    box('Workbench', (0.95, 0, 0.49), (0.5, 0.8, 0.10), (0.35, 0.33, 0.30))
    box('RedBlock', (0.93, 0.22, 0.60), (0.11, 0.11, 0.12), (0.90, 0.16, 0.08))
    box('YellowBlock', (0.93, -0.22, 0.60), (0.11, 0.11, 0.12), (0.95, 0.75, 0.08))


def add_pickup_scene(stage):
    """Solid table, two dynamic parcels, and their colored landing pads."""
    UsdGeom.Xform.Define(stage, '/World/PickupDemo')

    def cube(name, position, size, color, dynamic=False):
        prim = UsdGeom.Cube.Define(stage, '/World/PickupDemo/' + name)
        prim.CreateSizeAttr(float(size))
        prim.AddTranslateOp().Set(Gf.Vec3d(*position))
        prim.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
        if dynamic:
            body = UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim())
            body.CreateKinematicEnabledAttr(False)
            UsdPhysics.MassAPI.Apply(prim.GetPrim()).CreateMassAttr(0.12)
        return prim

    table = cube('Workbench', (0.67, 0., 0.49), 1., (0.34, 0.32, 0.29))
    table.AddScaleOp().Set(Gf.Vec3f(0.44, 0.88, 0.10))
    for side, y, color in (('left', .18, (.85, .12, .08)),
                            ('right', -.18, (.08, .35, .90))):
        cube(side.title()+'Parcel', (.63, y, .595), .10, color, dynamic=True)
        pad = cube(side.title()+'LandingPad', (.63, y + (.13 if y > 0 else -.13),
                                             .542), 1., color)
        pad.AddScaleOp().Set(Gf.Vec3f(.14, .14, .004))
