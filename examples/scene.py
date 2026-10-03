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


def add_lidar_scene(stage):
    """Three solid reference boxes for the 180-degree laser-slice example."""
    UsdGeom.Xform.Define(stage, '/World/LidarDemo')
    for name, position, color in (
        ('CenterBox', (1., 0., .30), (.95, .30, .12)),
        ('LeftBox', (1.15, .48, .30), (.20, .78, .36)),
        ('RightBox', (1.15, -.48, .30), (.22, .48, .95)),
    ):
        cube = UsdGeom.Cube.Define(stage, '/World/LidarDemo/'+name)
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*position))
        cube.AddScaleOp().Set(Gf.Vec3f(.2, .2, .28))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def add_lidar_room(stage):
    """The same walls and six solid obstacles as MuJoCo's live lidar room."""
    UsdGeom.Xform.Define(stage, '/World/LidarRoom')

    def box(name, position, half_size, color):
        cube = UsdGeom.Cube.Define(stage, '/World/LidarRoom/' + name)
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*position))
        cube.AddScaleOp().Set(Gf.Vec3f(*(2 * value for value in half_size)))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    box('FarWall', (4.7, 0, .6), (.06, 2., .6), (.42, .49, .54))
    box('LeftWall', (1.8, 2., .6), (2.9, .06, .6), (.39, .47, .53))
    box('RightWall', (1.8, -2., .6), (2.9, .06, .6), (.39, .47, .53))
    for name, position, size, color in (
        ('AmberCrate', (.9, .88, .34), (.18, .20, .34), (.98, .60, .17)),
        ('BlueCrate', (1.25, -.92, .32), (.20, .18, .32), (.16, .54, .9)),
        ('GreenCabinet', (2., 1.05, .55), (.22, .24, .55), (.24, .72, .45)),
        ('PurpleCrate', (2.55, -.96, .38), (.19, .19, .38), (.62, .38, .82)),
        ('RedCabinet', (3.35, 1.02, .51), (.23, .22, .51), (.9, .36, .31)),
        ('YellowCrate', (3.70, -.93, .32), (.17, .18, .32), (.9, .76, .25)),
    ):
        box(name, position, size, color)
