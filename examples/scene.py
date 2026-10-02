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
