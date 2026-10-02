"""Build a 3D model of the complete Building Bloch device.

The six printed parts are loaded from the STEP files one directory up and put
in their assembled positions; the purchased parts (stepper motors, omni wheels,
sphere, M5Stack, USB PD breakout) are modelled here from their nominal sizes.

    pip install cadquery
    python build_assembly.py            # writes building-bloch.step / .glb
    python build_assembly.py --check    # also prints part-to-part interference

Placement is worked out in the frame of the Base STEP file (Y up, mm, origin
at the centre of the bottom rim) and rotated to Z up on export.
"""
import math
import os
import sys

import cadquery as cq
import numpy as np
from cadquery import Color, Location, Plane, Vector
from OCP.BRepAdaptor import BRepAdaptor_Surface

HERE = os.path.dirname(os.path.abspath(__file__))
PARTS = os.path.join(HERE, "..", "23-0093 %s 01-10-2024.stp")

# Nominal dimensions (see README bill of materials and firmware config.h)
SPHERE_DIAM = 160.0
SPHERE_WALL = 1.0           # arrow is 158 long, so it spans the inside
WHEEL_DIAM = 58.0
WHEEL_WIDTH = 24.5          # fills the axle adapter from flange to tip
WHEEL_ROLLERS = 5           # per row, two rows offset by half a pitch
MOTOR_SIZE = 35.2           # NEMA 14: the base has 26 mm hole spacing, 22 mm boss
MOTOR_LENGTH = 28.0         # assumption, the motors are not in the BOM
M5_SIZE = 54.0
M5_CORE_HEIGHT = 16.0       # CoreS3 SE
M5_MODULE_HEIGHT = 13.2     # Stepmotor Driver Module v1.1

BASE_WALL_OUTER = 55.0      # motor face distance from origin along its normal
BASE_WALL_INNER = 52.0
BASE_TOP = 54.0
LID_THICKNESS = 2.5

PRINT = Color(0.42, 0.33, 0.62)
BLACK = Color(0.06, 0.06, 0.07)
RUBBER = Color(0.13, 0.13, 0.14)
STEEL = Color(0.75, 0.76, 0.78)
CLEAR = Color(0.85, 0.92, 1.0, 0.22)
SHELL = Color(0.82, 0.82, 0.80)
PCB = Color(0.05, 0.25, 0.45)


def load(name):
    return cq.importers.importStep(PARTS % name).val()


def on_axis(shape, origin, z_dir, x_dir):
    """Move a shape built around +Z at the origin onto an arbitrary axis."""
    return shape.moved(Plane(origin=tuple(origin), xDir=tuple(x_dir), normal=tuple(z_dir)).location)


def motor_mounts(base):
    """Find the three motor axes and screw patterns from the holes in the base.

    Returns (point on axis nearest the origin, unit axis, unit vector from the
    axis towards one screw hole) per motor.
    """
    bores, screws = [], []
    for face in base.Faces():
        if face.geomType() != "CYLINDER":
            continue
        cyl = BRepAdaptor_Surface(face.wrapped).Cylinder()
        d, p = cyl.Axis().Direction(), cyl.Location()
        axis, point = np.array([d.X(), d.Y(), d.Z()]), np.array([p.X(), p.Y(), p.Z()])
        if axis[1] < 0:
            axis = -axis
        if abs(axis[1] - 0.5) > 1e-3:      # motor faces lean 30 deg from vertical
            continue
        if abs(cyl.Radius() - 11.1) < 0.05:
            bores.append((point - axis * (point @ axis), axis))
        elif abs(cyl.Radius() - 1.55) < 0.05:
            screws.append((point, axis))
    mounts = []
    for foot, axis in bores:
        for point, saxis in screws:
            if abs(saxis @ axis) > 0.999:
                radial = (point - foot) - axis * ((point - foot) @ axis)
                mounts.append((foot, axis, radial / np.linalg.norm(radial)))
                break
    assert len(mounts) == 3, "expected three motor mounts in the base"
    return mounts


def make_motor():
    """NEMA 14 stepper, mounting face at z=0, shaft along +Z."""
    body = (cq.Workplane("XY").workplane(offset=-MOTOR_LENGTH)
            .rect(MOTOR_SIZE, MOTOR_SIZE).extrude(MOTOR_LENGTH).edges("|Z").chamfer(4.0))
    boss = cq.Workplane("XY").circle(11.0).extrude(2.0)
    shaft = cq.Workplane("XY").circle(2.5).extrude(20.0)
    return body.val(), boss.union(shaft).val()


def make_wheel():
    """58 mm double-row omni wheel, axis along +Z, centred on the origin.

    Returns (hub and plates, rollers).
    """
    r_out = WHEEL_DIAM / 2
    r_axle = 22.0                                   # roller axle circle
    half = r_axle * math.tan(math.radians(180 / WHEEL_ROLLERS)) - 3.2
    r_end = math.sqrt(r_out ** 2 - half ** 2) - r_axle
    # Barrel whose crown follows the wheel's outer circle
    roller = (cq.Workplane("XY").moveTo(-half, 0).lineTo(-half, r_end)
              .threePointArc((0, r_out - r_axle), (half, r_end)).lineTo(half, 0).close()
              .revolve(360, (0, 0, 0), (1, 0, 0)).val())
    row_z = WHEEL_WIDTH / 4 + 0.6
    body = cq.Workplane("XY").workplane(offset=-WHEEL_WIDTH / 2).circle(14.0).extrude(WHEEL_WIDTH)
    rollers = []
    for row, z in enumerate((-row_z, row_z)):
        for i in range(WHEEL_ROLLERS):
            ang = 360.0 / WHEEL_ROLLERS * (i + 0.5 * row)
            rollers.append(roller.moved(Location(Vector(0, r_axle, z)))
                           .rotate(Vector(0, 0, 0), Vector(0, 0, 1), ang))
            # fork between neighbouring rollers
            fork = (cq.Workplane("XY").workplane(offset=z - 4.5).center(0, 19.0).rect(5.0, 12.0).extrude(9.0)
                    .val().rotate(Vector(0, 0, 0), Vector(0, 0, 1), ang + 180.0 / WHEEL_ROLLERS))
            body = body.union(cq.Workplane(obj=fork))
    body = body.cut(cq.Workplane("XY").workplane(offset=-WHEEL_WIDTH).circle(6.1).extrude(2 * WHEEL_WIDTH))
    return body.val(), cq.Compound.makeCompound(rollers)


def make_sphere():
    """Two clear half shells, seam in the YZ plane, centred on the origin."""
    r = SPHERE_DIAM / 2
    shell = cq.Workplane("XY").sphere(r).cut(cq.Workplane("XY").sphere(r - SPHERE_WALL))
    lip = cq.Workplane("YZ").workplane(offset=-1.0).circle(r + 1.0).circle(r - SPHERE_WALL).extrude(2.0)
    return shell.union(lip).val()


def make_m5stack():
    """CoreS3 SE on a Stepmotor Driver Module, bottom of the stack at z=0."""
    module = cq.Workplane("XY").rect(M5_SIZE, M5_SIZE).extrude(M5_MODULE_HEIGHT).edges("|Z").fillet(5.0)
    core = (cq.Workplane("XY").workplane(offset=M5_MODULE_HEIGHT).rect(M5_SIZE, M5_SIZE)
            .extrude(M5_CORE_HEIGHT).edges("|Z").fillet(5.0).faces(">Z").edges().fillet(1.0))
    top = M5_MODULE_HEIGHT + M5_CORE_HEIGHT
    glass = (cq.Workplane("XY").workplane(offset=top).rect(48.0, 48.0).extrude(0.3).edges("|Z").fillet(3.0))
    return module.val(), core.val(), glass.val()


def make_pd_breakout():
    """Adafruit USB PD breakout, USB-C receptacle towards +X."""
    board = cq.Workplane("XY").rect(25.4, 17.8).extrude(1.6).edges("|Z").fillet(2.0)
    usb = cq.Workplane("XY").workplane(offset=1.6).center(9.0, 0).rect(7.4, 9.0).extrude(3.2).edges("|X").fillet(1.2)
    return board.val(), usb.val()


def build():
    """Return a list of (name, shape, colour) in the Base frame (Y up)."""
    y_axis, origin = Vector(0, 1, 0), Vector(0, 0, 0)
    base = load("Base")
    parts = [("base", base, PRINT)]

    # Lid drops onto the hexagonal opening in the top of the base
    parts.append(("lid", load("Lid - blank").moved(Location(Vector(0, BASE_TOP + LID_THICKNESS, 0))), PRINT))

    # Motors, axle adapters and wheels on the three slanted faces
    adapter = load("Axle adapter").rotate(origin, y_axis, -90)        # part X axis -> +Z
    motor_body, motor_shaft = make_motor()
    wheel_body, wheel_rollers = make_wheel()
    wheel_t = BASE_WALL_OUTER + 2.0 + WHEEL_WIDTH / 2                 # behind the adapter flange
    wheel_centres = []
    for i, (foot, axis, to_screw) in enumerate(motor_mounts(base), start=1):
        side = np.cross(axis, to_screw)
        diag = (to_screw + side) / math.sqrt(2)                       # screws sit on the diagonals
        parts.append((f"motor_{i}", on_axis(motor_body, foot + axis * BASE_WALL_INNER, axis, diag), BLACK))
        parts.append((f"motor_{i}_shaft", on_axis(motor_shaft, foot + axis * BASE_WALL_INNER, axis, diag), STEEL))
        parts.append((f"axle_adapter_{i}", on_axis(adapter, foot + axis * BASE_WALL_OUTER, axis, to_screw), PRINT))
        centre = foot + axis * wheel_t
        wheel_centres.append(centre)
        parts.append((f"omni_wheel_{i}", on_axis(wheel_body, centre, axis, to_screw), BLACK))
        parts.append((f"omni_wheel_{i}_rollers", on_axis(wheel_rollers, centre, axis, to_screw), RUBBER))

    # Sphere rests on the three wheels: its centre is on the vertical axis,
    # one sphere radius plus one wheel radius away from each wheel centre.
    reach = (SPHERE_DIAM + WHEEL_DIAM) / 2
    sphere_y = float(np.mean([c[1] + math.sqrt(reach ** 2 - c[0] ** 2 - c[2] ** 2) for c in wheel_centres]))
    centre = Vector(0, sphere_y, 0)
    parts.append(("sphere", make_sphere().moved(Location(centre)), CLEAR))
    # Arrow spans the inside diameter, pointing up (|0>, the state after reset)
    arrow = (load("Arrow").moved(Location(Vector(-(SPHERE_DIAM / 2 - SPHERE_WALL - 1.0), 0, 0)))
             .rotate(origin, Vector(0, 0, 1), 90).moved(Location(centre)))
    parts.append(("arrow", arrow, PRINT))

    # M5 housing plugs into the cut-out in the -X wall, opening facing away from the base
    def housing_frame(shape):
        return shape.rotate(origin, y_axis, 180).moved(Location(Vector(-BASE_WALL_OUTER, 0, 0)))

    parts.append(("m5_housing", housing_frame(load("M5 housing")), PRINT))
    # The frame sits on the 45 deg rim of the housing, the M5Stack on the frame's corner posts
    rim = Plane(origin=(34.14, 36.14, 0), xDir=(0, 0, 1), normal=(1, 1, 0)).location
    parts.append(("m5_frame", housing_frame(load("M5 frame").moved(rim)), PRINT))
    module, core, glass = make_m5stack()
    seat = Location(Vector(0, 0, -15.0))
    for name, shape, colour in (("stepmotor_driver_module", module, BLACK),
                                ("m5stack_cores3", core, SHELL),
                                ("m5stack_cores3_screen", glass, BLACK)):
        parts.append((name, housing_frame(shape.moved(seat).moved(rim)), colour))

    # USB PD breakout lies loose in the base between two motors (position approximate)
    board, usb = make_pd_breakout()
    spot = Location(Vector(-28.0, 6.0, 0)) * Location(Vector(0, 0, 0), Vector(1, 0, 0), -90)
    parts.append(("usb_pd_breakout", board.moved(spot), PCB))
    parts.append(("usb_pd_breakout_connector", usb.moved(spot), STEEL))
    return parts


def to_z_up(shape):
    return shape.rotate(Vector(0, 0, 0), Vector(1, 0, 0), 90)


def check(parts):
    """Print the overlap volume of every pair of parts that touch."""
    solid = [(n, s) for n, s, _ in parts]
    for i, (na, a) in enumerate(solid):
        for nb, b in solid[i + 1:]:
            ba, bb = a.BoundingBox(), b.BoundingBox()
            if (ba.xmin > bb.xmax or bb.xmin > ba.xmax or ba.ymin > bb.ymax or bb.ymin > ba.ymax
                    or ba.zmin > bb.zmax or bb.zmin > ba.zmax):
                continue
            try:
                v = a.intersect(b).Volume()
            except Exception:
                continue
            if v > 0.5:
                print(f"  overlap {na} / {nb}: {v:.1f} mm3")


def main():
    parts = build()
    if "--check" in sys.argv:
        check(parts)
    assy = cq.Assembly(name="building_bloch")
    for name, shape, colour in parts:
        assy.add(to_z_up(shape), name=name, color=colour)
    assy.export(os.path.join(HERE, "building-bloch.step"))
    assy.export(os.path.join(HERE, "building-bloch.glb"), tolerance=0.05, angularTolerance=0.2)
    bb = assy.toCompound().BoundingBox()
    print(f"{len(parts)} parts, {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm")


if __name__ == "__main__":
    main()
