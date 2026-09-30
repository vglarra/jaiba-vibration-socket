import bpy, bmesh, math, os
from mathutils import Vector

print("=== TPU HEX SOCKET (piezo vibration damper) ===")

# ---- PARAMETERS (all mm) ----------------------------------------------
# Values marked [from tray script] were read from the tray's Blender script.
PAD_FLAT        = 78.0   # [INNER_TILE_WIDTH]  pad, flat-to-flat
PAD_HEIGHT      = 5.0    # [INNER_TILE_HEIGHT] pad height -> depth of the underside recess
WALL_T          = 3.0    # socket wall (2 x 3 = the 6mm gap between pads)
FLOOR_T         = 3.0    # floor slab between the underside recess and the cup
TOTAL_H         = 26.0   # overall socket height (matches tray WALL_HEIGHT)
FIT_CLEARANCE   = 0.0    # extra room PER SIDE around the pad (0.1-0.2 is typical for TPU)
                         # NOTE: raising this grows the OUTER hex past the 84mm tile pitch,
                         # so neighbouring sockets would overlap by 2*clearance.

# ---- WIRE HOLE (same position/size as the tray's per-tile hole) ----------
WIRE_HOLE            = True
WIRE_HOLE_DIAMETER   = 12.0   # [WIRE_HOLE_DIAMETER]
WIRE_HOLE_OFFSET     = 28.0   # [HOLE_RADIUS] distance from hex center to hole center
WIRE_HOLE_ANGLE      = 180    # [WIRE_HOLE_ANGLE] 180 = toward the left flat
WIRE_HOLE_SEGMENTS   = 32
WIRE_CUT_OVERSHOOT   = 1.0    # cutter pokes this far past the floor slab into the
                              # open recess/cup, so it never sits on a real face

CENTER          = (0.0, 0.0)
OBJ_NAME        = "TPU_Socket"
EXPORT_STL      = True
STL_NAME        = "tpu_hex_socket.stl"

# ---- DERIVED ------------------------------------------------------------
INNER_FLAT   = PAD_FLAT + 2 * FIT_CLEARANCE
OUTER_FLAT   = INNER_FLAT + 2 * WALL_T          # 84.0 with defaults
Z_RECESS_TOP = PAD_HEIGHT                       # 5.0  (underside recess ceiling)
Z_FLOOR_TOP  = Z_RECESS_TOP + FLOOR_T           # 8.0  (cup floor surface)
CUP_DEPTH    = TOTAL_H - Z_FLOOR_TOP            # 18.0 (interior height)
TILE_PITCH   = 84.0                             # tray HEX_FLAT_WIDTH (horizontal pitch)
WIRE_R       = WIRE_HOLE_DIAMETER / 2.0

print(f"Pad:      {PAD_FLAT:.2f} flat-to-flat (side {PAD_FLAT/math.sqrt(3):.3f}), {PAD_HEIGHT:.1f} tall")
print(f"Interior: {INNER_FLAT:.2f} flat-to-flat")
print(f"Outer:    {OUTER_FLAT:.2f} flat-to-flat (side {OUTER_FLAT/math.sqrt(3):.3f})")
print(f"Z levels: 0 | recess ceiling {Z_RECESS_TOP:.1f} | cup floor {Z_FLOOR_TOP:.1f} | top {TOTAL_H:.1f}")
print(f"Cup interior depth: {CUP_DEPTH:.1f}")
if OUTER_FLAT > TILE_PITCH + 1e-6:
    print(f"WARNING: outer {OUTER_FLAT:.2f} > tile pitch {TILE_PITCH:.2f}: adjacent sockets will overlap")
if CUP_DEPTH <= 0:
    raise ValueError("TOTAL_H too small for pad height + floor thickness")
if WIRE_HOLE and WIRE_HOLE_OFFSET + WIRE_R > INNER_FLAT / 2.0 - 1.0:
    print("WARNING: wire hole comes within 1mm of (or past) the interior wall")

# ---- HELPERS ------------------------------------------------------------
def hex_corners(cx, cy, flat_width):
    """Same convention as the tray script: corners at 30+60*i degrees
    (pointy-top, flats facing +/-X), circumradius = flat/sqrt(3)."""
    s = flat_width / math.sqrt(3)
    return [(cx + s * math.cos(math.radians(30 + 60 * i)),
             cy + s * math.sin(math.radians(30 + 60 * i))) for i in range(6)]

def cleanup():
    for name in (OBJ_NAME, "Cut_WireHole"):
        obj = bpy.data.objects.get(name)
        if obj:
            bpy.data.objects.remove(obj, do_unlink=True)
    for name in (OBJ_NAME + "_mesh", "Cut_WireHole_mesh"):
        m = bpy.data.meshes.get(name)
        if m:
            bpy.data.meshes.remove(m)

def mesh_stats(obj):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.normal_update()
    nm = sum(1 for e in bm.edges if not e.is_manifold)
    vol = bm.calc_volume(signed=True)
    bm.free()
    return nm, vol

def build_socket():
    mesh = bpy.data.meshes.new(OBJ_NAME + "_mesh")
    obj = bpy.data.objects.new(OBJ_NAME, mesh)
    bpy.context.collection.objects.link(obj)

    bm = bmesh.new()
    cx, cy = CENTER
    def ring(flat, z):
        return [bm.verts.new((x, y, z)) for x, y in hex_corners(cx, cy, flat)]

    o0 = ring(OUTER_FLAT, 0.0)            # outer, bottom
    o1 = ring(OUTER_FLAT, TOTAL_H)        # outer, top
    i0 = ring(INNER_FLAT, 0.0)            # recess mouth (underside)
    i1 = ring(INNER_FLAT, Z_RECESS_TOP)   # recess ceiling
    i2 = ring(INNER_FLAT, Z_FLOOR_TOP)    # cup floor
    i3 = ring(INNER_FLAT, TOTAL_H)        # cup rim

    for i in range(6):
        j = (i + 1) % 6
        bm.faces.new([o0[i], o0[j], o1[j], o1[i]])   # outer wall
        bm.faces.new([o0[i], i0[i], i0[j], o0[j]])   # bottom rim (3mm ring)
        bm.faces.new([i0[i], i0[j], i1[j], i1[i]])   # underside recess wall (5mm)
        bm.faces.new([i2[i], i2[j], i3[j], i3[i]])   # cup wall (18mm)
        bm.faces.new([o1[i], o1[j], i3[j], i3[i]])   # top rim (3mm ring)
    bm.faces.new(i1)   # recess ceiling (underside of the floor)
    bm.faces.new(i2)   # cup floor (top of the floor)

    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    bm.normal_update()
    bm.to_mesh(mesh)
    bm.free()
    mesh.validate()
    mesh.update()

    hex_area = lambda f: (math.sqrt(3) / 2.0) * f * f
    expected = (hex_area(OUTER_FLAT) - hex_area(INNER_FLAT)) * TOTAL_H + hex_area(INNER_FLAT) * FLOOR_T
    nm, vol = mesh_stats(obj)
    print(f"Body (before wire hole): non-manifold edges {nm} (expect 0), "
          f"volume {vol:.1f} mm^3 (expected {expected:.1f}) "
          f"{'OK' if abs(vol - expected) < 1.0 else 'MISMATCH - normals may be inverted'}")
    return obj, vol

def make_wire_cutter():
    """Vertical cylinder spanning only the 3mm floor slab plus a small
    overshoot into the open recess below and the open cup above."""
    ang = math.radians(WIRE_HOLE_ANGLE)
    hx = CENTER[0] + WIRE_HOLE_OFFSET * math.cos(ang)
    hy = CENTER[1] + WIRE_HOLE_OFFSET * math.sin(ang)
    depth = FLOOR_T + 2 * WIRE_CUT_OVERSHOOT
    zc = (Z_RECESS_TOP + Z_FLOOR_TOP) / 2.0
    print(f"Wire hole at ({hx:.3f}, {hy:.3f}), diameter {WIRE_HOLE_DIAMETER:.1f}, "
          f"through floor Z={Z_RECESS_TOP:.1f}..{Z_FLOOR_TOP:.1f}")

    mesh = bpy.data.meshes.new("Cut_WireHole_mesh")
    cutter = bpy.data.objects.new("Cut_WireHole", mesh)
    bpy.context.collection.objects.link(cutter)
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False,
                          segments=WIRE_HOLE_SEGMENTS,
                          radius1=WIRE_R, radius2=WIRE_R, depth=depth)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    bm.to_mesh(mesh)
    bm.free()
    cutter.location = Vector((hx, hy, zc))
    bpy.context.view_layer.update()
    return cutter

def cut_wire_hole(obj, vol_before):
    cutter = make_wire_cutter()
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    mod = obj.modifiers.new("WireHole", 'BOOLEAN')
    mod.operation = 'DIFFERENCE'
    mod.object = cutter
    mod.solver = 'EXACT'
    try:
        bpy.ops.object.modifier_apply(modifier=mod.name)
    except Exception as e:
        print(f"WIRE HOLE FAILED: {e}")
        obj.modifiers.remove(mod)
    cm = cutter.data
    bpy.data.objects.remove(cutter, do_unlink=True)
    bpy.data.meshes.remove(cm)

    nm, vol = mesh_stats(obj)
    n = WIRE_HOLE_SEGMENTS
    hole_area = 0.5 * n * WIRE_R ** 2 * math.sin(2 * math.pi / n)
    expected_removed = hole_area * FLOOR_T
    removed = vol_before - vol
    ok = nm == 0 and abs(removed - expected_removed) < 2.0
    print(f"After wire hole: non-manifold edges {nm} (expect 0), removed {removed:.1f} mm^3 "
          f"(expected {expected_removed:.1f}) {'OK' if ok else 'CHECK THIS'}")

def export_stl(obj):
    base = bpy.data.filepath
    folder = os.path.dirname(base) if base else os.path.expanduser("~")
    path = os.path.join(folder, STL_NAME)
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    try:
        bpy.ops.wm.stl_export(filepath=path, export_selected_objects=True)
    except Exception:
        bpy.ops.export_mesh.stl(filepath=path, use_selection=True)
    print(f"Exported: {path}")

cleanup()
socket, body_volume = build_socket()
if WIRE_HOLE:
    cut_wire_hole(socket, body_volume)
if EXPORT_STL:
    export_stl(socket)
print("=== DONE ===")