import bpy, bmesh, math, os
from mathutils import Vector
from mathutils.geometry import tessellate_polygon

print("=== TPU HEX SOCKET (piezo vibration damper) ===")

# ---- PARAMETERS (all mm) ----------------------------------------------
# Values marked [from tray script] were read from the tray's Blender script.
PAD_FLAT        = 78.0   # [INNER_TILE_WIDTH]  pad, flat-to-flat
PAD_HEIGHT      = 5.0    # [INNER_TILE_HEIGHT] pad height -> depth of the underside recess
WALL_T          = 3.0    # socket wall (2 x 3 = the 6mm gap between pads)
FLOOR_T         = 3.0    # floor slab between the underside recess and the cup
TOTAL_H         = 26.0   # overall socket height (matches tray WALL_HEIGHT)
FIT_CLEARANCE   = 0.0    # extra room PER SIDE around the pad (see README)

# ---- SENSOR POCKET + SUPPORT FILL ----------------------------------------
FILL                = True    # False = plain cup (no ring / honeycomb)
SENSOR_D            = 67.5    # piezo sensor puck diameter
SENSOR_H            = 10.0    # puck height -> top sits flush with TOTAL_H
SENSOR_CLEARANCE    = 0.2     # per side, pocket radius = SENSOR_D/2 + this
RING_T              = 2.0     # retaining ring wall around the sensor
FILL_WALL_T         = 2.0     # honeycomb wall thickness (also min solid band
                              # kept between cells and the ring / wire bore)
CELL_INNER          = 8.0     # honeycomb cell width, flat-to-flat (open space)
MIN_MOAT            = 1.0     # sanity check: ring must stay this far from hex wall

# ---- WIRE HOLE (same position as the tray's per-tile hole) ----------------
WIRE_HOLE            = True
WIRE_HOLE_DIAMETER   = 12.0   # [WIRE_HOLE_DIAMETER] hole through the 3mm floor
WIRE_BORE_DIAMETER   = 10.0   # bore through the honeycomb above the floor. Smaller
                              # than the floor hole on purpose: a 12mm bore at 28mm
                              # offset reaches r=34.0 vs the pocket wall at 33.95,
                              # leaving a 0.05mm sliver against the sensor ring.
WIRE_HOLE_OFFSET     = 28.0   # [HOLE_RADIUS] distance from hex center to hole center
WIRE_HOLE_ANGLE      = 180    # [WIRE_HOLE_ANGLE] 180 = toward the left flat
WIRE_SEGMENTS        = 32

SHOW_SENSOR_PROXY = True      # wireframe puck in the viewport (not exported)
CENTER          = (0.0, 0.0)
OBJ_NAME        = "TPU_Socket"
EXPORT_STL      = True
STL_NAME        = "tpu_hex_socket.stl"
RING_SEGMENTS   = 72

# ---- DERIVED ------------------------------------------------------------
INNER_FLAT   = PAD_FLAT + 2 * FIT_CLEARANCE
OUTER_FLAT   = INNER_FLAT + 2 * WALL_T          # 84.0 with defaults
Z_RECESS_TOP = PAD_HEIGHT                       # 5.0
Z_FLOOR_TOP  = Z_RECESS_TOP + FLOOR_T           # 8.0
Z_BED        = TOTAL_H - SENSOR_H               # 16.0 (sensor bottom / honeycomb top)
POCKET_R     = SENSOR_D / 2.0 + SENSOR_CLEARANCE
RING_OUT_R   = POCKET_R + RING_T
WIRE_R       = WIRE_HOLE_DIAMETER / 2.0
BORE_R       = WIRE_BORE_DIAMETER / 2.0
TILE_PITCH   = 84.0

_a = math.radians(WIRE_HOLE_ANGLE)
WIRE_CX = CENTER[0] + WIRE_HOLE_OFFSET * math.cos(_a)
WIRE_CY = CENTER[1] + WIRE_HOLE_OFFSET * math.sin(_a)

print(f"Outer hex {OUTER_FLAT:.2f} / interior {INNER_FLAT:.2f} flat-to-flat, height {TOTAL_H:.1f}")
print(f"Z: recess 0..{Z_RECESS_TOP:.1f} | floor ..{Z_FLOOR_TOP:.1f} | honeycomb ..{Z_BED:.1f} | top {TOTAL_H:.1f}")
if FILL:
    print(f"Sensor pocket r={POCKET_R:.2f}, ring outer r={RING_OUT_R:.2f}, "
          f"moat at flats {INNER_FLAT/2 - RING_OUT_R:.2f}mm")
if OUTER_FLAT > TILE_PITCH + 1e-6:
    print(f"WARNING: outer {OUTER_FLAT:.2f} > tile pitch {TILE_PITCH:.2f}: adjacent sockets will overlap")
if Z_BED <= Z_FLOOR_TOP:
    raise ValueError("Sensor too tall: no room left above the floor for the support zone")
if FILL and RING_OUT_R > INNER_FLAT / 2.0 - MIN_MOAT:
    raise ValueError("Sensor ring is too close to (or past) the hex wall")
if FILL and WIRE_HOLE and WIRE_HOLE_OFFSET + BORE_R > POCKET_R - 0.5:
    raise ValueError("Wire bore comes within 0.5mm of the pocket wall - reduce WIRE_BORE_DIAMETER")

# ---- GEOMETRY HELPERS ------------------------------------------------------
def hex_corners(cx, cy, flat_width):
    """Tray convention: corners at 30+60*i deg (pointy-top, flats face +/-X)."""
    s = flat_width / math.sqrt(3)
    return [(cx + s * math.cos(math.radians(30 + 60 * i)),
             cy + s * math.sin(math.radians(30 + 60 * i))) for i in range(6)]

def circle_pts(cx, cy, r, n):
    return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n))
            for i in range(n)]

def poly_area(pts):
    a = 0.0
    for i in range(len(pts)):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % len(pts)]
        a += x0 * y1 - x1 * y0
    return abs(a) / 2.0

def dist_point_to_convex_poly(p, poly):
    """0 if inside, else distance to nearest edge."""
    n = len(poly)
    inside = True
    best = 1e9
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        ex, ey = x1 - x0, y1 - y0
        cross = ex * (p[1] - y0) - ey * (p[0] - x0)
        if cross < 0:
            inside = False
        t = max(0.0, min(1.0, ((p[0] - x0) * ex + (p[1] - y0) * ey) / (ex * ex + ey * ey)))
        best = min(best, math.hypot(p[0] - (x0 + t * ex), p[1] - (y0 + t * ey)))
    return 0.0 if inside else best

def honeycomb_cells():
    """Whole hex cells only (no clipping against the circle). Cells that
    would come within FILL_WALL_T of the ring or the wire bore are dropped,
    which leaves a solid band there instead of ragged partial cells."""
    pitch = CELL_INNER + FILL_WALL_T
    s_c = pitch / math.sqrt(3)
    cells = []
    for row in range(-10, 11):
        for col in range(-10, 11):
            cx = CENTER[0] + col * pitch + (pitch / 2.0 if row % 2 else 0.0)
            cy = CENTER[1] + row * 1.5 * s_c
            pts = hex_corners(cx, cy, CELL_INNER)
            if max(math.hypot(x - CENTER[0], y - CENTER[1]) for x, y in pts) > POCKET_R - FILL_WALL_T:
                continue
            if WIRE_HOLE and dist_point_to_convex_poly((WIRE_CX, WIRE_CY), pts) < BORE_R + FILL_WALL_T:
                continue
            cells.append(pts)
    return cells

def cleanup():
    for name in (OBJ_NAME, "SensorProxy"):
        obj = bpy.data.objects.get(name)
        if obj:
            bpy.data.objects.remove(obj, do_unlink=True)
    for name in (OBJ_NAME + "_mesh", "SensorProxy_mesh"):
        m = bpy.data.meshes.get(name)
        if m:
            bpy.data.meshes.remove(m)

def mesh_stats(obj):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.normal_update()
    nm = sum(1 for e in bm.edges if not e.is_manifold)
    za = sum(1 for f in bm.faces if f.calc_area() < 1e-6)
    vol = bm.calc_volume(signed=True)
    bm.free()
    return nm, za, vol

# ---- BUILD ---------------------------------------------------------------
def build_socket():
    mesh = bpy.data.meshes.new(OBJ_NAME + "_mesh")
    obj = bpy.data.objects.new(OBJ_NAME, mesh)
    bpy.context.collection.objects.link(obj)
    bm = bmesh.new()
    cx, cy = CENTER

    def loop(pts, z):
        return [bm.verts.new((x, y, z)) for x, y in pts]

    def quads(a, b):
        """Quad strip between two equal-length loops."""
        n = len(a)
        for i in range(n):
            j = (i + 1) % n
            bm.faces.new([a[i], a[j], b[j], b[i]])

    def face_with_holes(outer, holes):
        """Planar face (all verts at one Z) with optional holes, tessellated by
        Blender. Winding is fixed globally by recalc_face_normals afterwards."""
        def signed(vs):
            return sum(vs[i].co.x * vs[(i + 1) % len(vs)].co.y - vs[(i + 1) % len(vs)].co.x * vs[i].co.y
                       for i in range(len(vs)))
        loops = [list(outer) if signed(outer) > 0 else list(reversed(outer))]
        for h in holes:
            loops.append(list(h) if signed(h) < 0 else list(reversed(h)))
        flat = [v for lp in loops for v in lp]
        polys = [[Vector((v.co.x, v.co.y, 0.0)) for v in lp] for lp in loops]
        for a, b, c in tessellate_polygon(polys):
            bm.faces.new([flat[a], flat[b], flat[c]])

    # -- hex shell: outer wall, rims, recess wall, cup wall ---------------------
    o0 = loop(hex_corners(cx, cy, OUTER_FLAT), 0.0)
    o1 = loop(hex_corners(cx, cy, OUTER_FLAT), TOTAL_H)
    i0 = loop(hex_corners(cx, cy, INNER_FLAT), 0.0)
    i1 = loop(hex_corners(cx, cy, INNER_FLAT), Z_RECESS_TOP)
    i2 = loop(hex_corners(cx, cy, INNER_FLAT), Z_FLOOR_TOP)
    i3 = loop(hex_corners(cx, cy, INNER_FLAT), TOTAL_H)
    quads(o0, o1)                              # outer wall
    quads([o0[i] for i in range(6)], i0)       # bottom rim  (o0 -> i0)
    quads(i0, i1)                              # underside recess wall
    quads(i2, i3)                              # cup wall
    quads(o1, i3)                              # top rim
    # (quads() winding for rims is corrected by recalc_face_normals below)

    # -- wire hole through the 3mm floor -----------------------------------------
    hole_pts = circle_pts(WIRE_CX, WIRE_CY, WIRE_R, WIRE_SEGMENTS) if WIRE_HOLE else None
    fh5 = loop(hole_pts, Z_RECESS_TOP) if WIRE_HOLE else None
    fh8 = loop(hole_pts, Z_FLOOR_TOP) if WIRE_HOLE else None
    face_with_holes(i1, [fh5] if WIRE_HOLE else [])      # recess ceiling
    if WIRE_HOLE:
        quads(fh8, fh5)                                   # floor hole wall

    voids = []
    if not FILL:
        face_with_holes(i2, [fh8] if WIRE_HOLE else [])   # plain cup floor
    else:
        voids = honeycomb_cells()
        N = RING_SEGMENTS
        co8  = loop(circle_pts(cx, cy, RING_OUT_R, N), Z_FLOOR_TOP)
        co26 = loop(circle_pts(cx, cy, RING_OUT_R, N), TOTAL_H)
        ci26 = loop(circle_pts(cx, cy, POCKET_R, N), TOTAL_H)
        ci16 = loop(circle_pts(cx, cy, POCKET_R, N), Z_BED)

        face_with_holes(i2, [co8])          # moat floor (hex minus ring footprint)
        quads(co8, co26)                    # ring outer wall
        quads(co26, ci26)                   # ring top
        quads(ci26, ci16)                   # ring inner wall (sensor side)

        bed_holes = []
        for pts in voids:                   # honeycomb cells
            v16 = loop(pts, Z_BED)
            v8 = loop(pts, Z_FLOOR_TOP)
            quads(v16, v8)                  # cell walls
            bm.faces.new(v8)                # cell floor
            bed_holes.append(v16)
        if WIRE_HOLE:                       # wire bore through the honeycomb
            bore_pts = circle_pts(WIRE_CX, WIRE_CY, BORE_R, WIRE_SEGMENTS)
            b16 = loop(bore_pts, Z_BED)
            b8 = loop(bore_pts, Z_FLOOR_TOP)
            quads(b16, b8)                  # bore wall
            quads(b8, fh8)                  # 1mm ledge between 10mm bore and 12mm floor hole
            bed_holes.append(b16)
        face_with_holes(ci16, bed_holes)    # honeycomb wall tops = sensor bed

    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    bm.normal_update()
    bm.to_mesh(mesh)
    bm.free()
    mesh.validate()
    mesh.update()

    # -- verification against analytic volume ---------------------------------------
    hex_area = lambda f: (math.sqrt(3) / 2.0) * f * f
    expected = (hex_area(OUTER_FLAT) - hex_area(INNER_FLAT)) * TOTAL_H + hex_area(INNER_FLAT) * FLOOR_T
    if WIRE_HOLE:
        expected -= poly_area(hole_pts) * FLOOR_T
    solid_frac = None
    if FILL:
        a_co = poly_area(circle_pts(cx, cy, RING_OUT_R, RING_SEGMENTS))
        a_ci = poly_area(circle_pts(cx, cy, POCKET_R, RING_SEGMENTS))
        a_void = sum(poly_area(p) for p in voids)
        a_bore = poly_area(circle_pts(WIRE_CX, WIRE_CY, BORE_R, WIRE_SEGMENTS)) if WIRE_HOLE else 0.0
        expected += (a_co - a_void - a_bore) * (Z_BED - Z_FLOOR_TOP) + (a_co - a_ci) * (TOTAL_H - Z_BED)
        solid_frac = (a_ci - a_void - a_bore) / a_ci
    nm, za, vol = mesh_stats(obj)
    print(f"Honeycomb cells: {len(voids)}" + (f", solid fraction inside pocket {solid_frac*100:.0f}%" if FILL else ""))
    print(f"Non-manifold edges: {nm} (expect 0), zero-area faces: {za} (expect 0)")
    print(f"Volume: {vol:.1f} mm^3 (expected {expected:.1f}) "
          f"{'OK' if abs(vol - expected) < 1.0 else 'MISMATCH - check normals / topology'}")
    return obj

def add_sensor_proxy():
    mesh = bpy.data.meshes.new("SensorProxy_mesh")
    obj = bpy.data.objects.new("SensorProxy", mesh)
    bpy.context.collection.objects.link(obj)
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=64,
                          radius1=SENSOR_D / 2.0, radius2=SENSOR_D / 2.0, depth=SENSOR_H)
    bm.to_mesh(mesh)
    bm.free()
    obj.location = Vector((CENTER[0], CENTER[1], Z_BED + SENSOR_H / 2.0))
    obj.display_type = 'WIRE'
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
if FILL and SHOW_SENSOR_PROXY:
    add_sensor_proxy()
if EXPORT_STL:
    export_stl(socket)
print("=== DONE ===")