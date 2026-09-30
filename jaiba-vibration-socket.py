import bpy, bmesh, math, os

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

print(f"Pad:      {PAD_FLAT:.2f} flat-to-flat (side {PAD_FLAT/math.sqrt(3):.3f}), {PAD_HEIGHT:.1f} tall")
print(f"Interior: {INNER_FLAT:.2f} flat-to-flat")
print(f"Outer:    {OUTER_FLAT:.2f} flat-to-flat (side {OUTER_FLAT/math.sqrt(3):.3f})")
print(f"Z levels: 0 | recess ceiling {Z_RECESS_TOP:.1f} | cup floor {Z_FLOOR_TOP:.1f} | top {TOTAL_H:.1f}")
print(f"Cup interior depth: {CUP_DEPTH:.1f}")
if OUTER_FLAT > TILE_PITCH + 1e-6:
    print(f"WARNING: outer {OUTER_FLAT:.2f} > tile pitch {TILE_PITCH:.2f}: adjacent sockets will overlap")
if CUP_DEPTH <= 0:
    raise ValueError("TOTAL_H too small for pad height + floor thickness")

# ---- HELPERS ------------------------------------------------------------
def hex_corners(cx, cy, flat_width):
    """Same convention as the tray script: corners at 30+60*i degrees
    (pointy-top, flats facing +/-X), circumradius = flat/sqrt(3)."""
    s = flat_width / math.sqrt(3)
    return [(cx + s * math.cos(math.radians(30 + 60 * i)),
             cy + s * math.sin(math.radians(30 + 60 * i))) for i in range(6)]

def cleanup():
    obj = bpy.data.objects.get(OBJ_NAME)
    if obj:
        bpy.data.objects.remove(obj, do_unlink=True)
    m = bpy.data.meshes.get(OBJ_NAME + "_mesh")
    if m:
        bpy.data.meshes.remove(m)

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

    nonmanifold = sum(1 for e in bm.edges if not e.is_manifold)
    volume = bm.calc_volume(signed=True)
    bm.to_mesh(mesh)
    bm.free()
    mesh.validate()
    mesh.update()

    hex_area = lambda f: (math.sqrt(3) / 2.0) * f * f
    expected = (hex_area(OUTER_FLAT) - hex_area(INNER_FLAT)) * TOTAL_H + hex_area(INNER_FLAT) * FLOOR_T
    print(f"Non-manifold edges: {nonmanifold} (expect 0)")
    print(f"Volume: {volume:.1f} mm^3 (expected {expected:.1f}) "
          f"{'OK' if abs(volume - expected) < 1.0 else 'MISMATCH - normals may be inverted'}")
    return obj

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
socket = build_socket()
if EXPORT_STL:
    export_stl(socket)
print("=== DONE ===")