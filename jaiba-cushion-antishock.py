import bpy, bmesh, math, os
from mathutils import Vector
from mathutils.geometry import tessellate_polygon

print("=== COOKIE CUSHION v2 (multi-row diamond lattice, braced break-away foot) ===")

# The disc stands on its rim: axis along Y, you press along Y.
# The cushion is real geometry: two thin FACE PLATES (optional) joined by an
# X-braced DIAMOND LATTICE, ROWS diamonds deep across the thickness. Every web is
# a vertical sheet, so it prints as plain perimeters (no slicer infill).
# A solid FOOT cradles the curved underside and is cut off afterwards.
#
# Why wide/flat diamonds: a diamond's struts bend under load. Short, steep struts
# are stiff; long, shallow ones are soft. So more ROWS and a WIDE CELL_W = softer,
# while smaller diamonds with the same web thickness get STIFFER.
# The script prints an estimated stiffness vs the reference cookie (see estimate()).

# ---- CUSHION PARAMETERS (all mm) --------------------------------------------
DIAMETER   = 64.0   # disc diameter (= height of the standing disc)
THICKNESS  = 17.0   # disc thickness along Y = cushion travel direction
ROWS       = 2      # diamonds stacked across the thickness
CELL_W     = 8.0    # diamond width along X (wider = softer)
WEB_T      = 0.9    # web thickness: 0.45 = 1 line, 0.9 = 2 perimeters
FACE_T     = 0.9    # face plate thickness; 0 = open faces (webs end in small feet)
# One cookie per entry (keys: rows, cell_w, web, face); omitted keys use the above.
VARIANTS = [
    dict(rows=2, cell_w=8,  web=0.9,  face=0.9),
    dict(rows=2, cell_w=8,  web=0.6,  face=0.45),
    dict(rows=2, cell_w=10, web=0.45, face=0.45),   # the printed estimate shows how soft each one should be
]

# ---- WIRING NOTCH -----------------------------------------------------------
# Half-stadium slot cut through the whole thickness, open to the rim: a semicircular
# inner end of diameter WIRE_D, straight sides running out through the perimeter.
WIRE_HOLE     = True
WIRE_D        = 10.0   # notch width = diameter of the rounded end
WIRE_DEPTH    = 10.0   # how far the notch reaches in from the rim (to the innermost point).
                       # 10 = a 10 mm circle that just touches the rim, opened out to the rim.
WIRE_ANGLE    = 0.0    # position around the rim in degrees from the TOP, + toward +X.
                       # Keep within about +-40: the top prints cleanly, a side notch would
                       # need to bridge its roof.

# ---- FOOT PARAMETERS --------------------------------------------------------
FOOT_H        = 2.0    # foot thickness under the disc's lowest point
GAP           = 0.2    # vertical gap under the disc = one layer height
GAP_SIDE      = 0.4    # clearance on the steep sides of the cradle
SUPPORT_ANGLE = 65.0   # cradle wraps the rim up to this angle from the bottom
FLARE_X       = 3.0    # cradle shelf beyond the arc (extra footprint)
FLANGE        = 6.0    # base plate sticks out this far past each flat face
FLANGE_T      = 1.6    # base plate thickness
BRACES        = True   # thin walls beside each face, stop the disc swaying in Y
BRACE_H       = 24.0   # brace height
BRACE_T       = 1.2    # brace thickness
BRACE_GAP     = 0.35   # clearance between brace and the disc face
GUSSET_XS     = (-12.0, 0.0, 12.0)   # triangular ribs behind each brace
GUSSET_T      = 1.2
BRACE_HALF_W  = 16.0   # brace half width in X
CIRCLE_SEG    = 180
ARC_SEG       = 64
SPACING       = 70.0   # viewport spacing between variants (not exported)
EXPORT_STL    = True

# ---- GEOMETRY HELPERS -------------------------------------------------------
def poly_area(pts):
    a = 0.0
    for i in range(len(pts)):
        x0, y0 = pts[i]; x1, y1 = pts[(i + 1) % len(pts)]
        a += x0 * y1 - x1 * y0
    return a / 2.0                                    # signed, CCW positive

def inset_convex(poly, dists):
    """Move edge i (poly[i] -> poly[i+1]) inward by dists[i]; poly must be CCW."""
    n = len(poly); lines = []
    for i in range(n):
        (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % n]
        dx, dy = x1 - x0, y1 - y0; L = math.hypot(dx, dy)
        nx, ny = -dy / L, dx / L
        lines.append(((x0 + nx * dists[i], y0 + ny * dists[i]), (dx / L, dy / L)))
    out = []
    for i in range(n):
        (p, d), (q, e) = lines[i - 1], lines[i]
        den = d[0] * e[1] - d[1] * e[0]
        t = ((q[0] - p[0]) * e[1] - (q[1] - p[1]) * e[0]) / den
        out.append((p[0] + d[0] * t, p[1] + d[1] * t))
    if poly_area(out) <= 0.05:
        raise ValueError("Web too thick for these cells: a void collapsed. Widen CELL_W or thin WEB_T.")
    return out

def prism(bm, pts, axis, a0, a1):
    """Extrude a closed 2-D profile. axis 'z': profile (x,y); 'y': profile (x,z); 'x': profile (y,z)."""
    def P(u, v, w):
        return {'z': (u, v, w), 'y': (u, w, v), 'x': (w, u, v)}[axis]
    lo = [bm.verts.new(P(u, v, a0)) for u, v in pts]
    hi = [bm.verts.new(P(u, v, a1)) for u, v in pts]
    n = len(pts)
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new([lo[i], lo[j], hi[j], hi[i]])
    for loop in (lo, hi):
        for i0, i1, i2 in tessellate_polygon([[Vector((u, v, 0.0)) for u, v in pts]]):
            bm.faces.new([loop[i0], loop[i1], loop[i2]])

def make_object(name, build):
    mesh = bpy.data.meshes.new(name + "_mesh")
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    bm = bmesh.new(); build(bm)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    bm.normal_update(); bm.to_mesh(mesh); bm.free()
    mesh.validate(); mesh.update()
    return obj

def remove(obj):
    me = obj.data
    bpy.data.objects.remove(obj, do_unlink=True)
    if me.users == 0:
        bpy.data.meshes.remove(me)

def boolean(obj, other, op):
    m = obj.modifiers.new("b_" + other.name, 'BOOLEAN')
    m.operation = op; m.operand_type = 'OBJECT'; m.object = other; m.solver = 'EXACT'

def evaluate(obj, name):
    bpy.context.view_layer.update()
    mesh = bpy.data.meshes.new_from_object(obj.evaluated_get(bpy.context.evaluated_depsgraph_get()))
    mesh.name = name + "_mesh"
    bm = bmesh.new(); bm.from_mesh(mesh)
    bmesh.ops.dissolve_degenerate(bm, dist=1e-5, edges=bm.edges[:])
    bad = [f for f in bm.faces if f.calc_area() < 1e-7]
    if bad: bmesh.ops.delete(bm, geom=bad, context='FACES')
    loose = [v for v in bm.verts if not v.link_faces]
    if loose: bmesh.ops.delete(bm, geom=loose, context='VERTS')
    bm.to_mesh(mesh); bm.free()
    out = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(out)
    return out

def stats(obj):
    bm = bmesh.new(); bm.from_mesh(obj.data); bm.normal_update()
    nm = sum(1 for e in bm.edges if not e.is_manifold)
    vol = bm.calc_volume(signed=True); bm.free()
    return nm, vol

def fmt(v):
    return f"{v:g}".replace(".", "p")

# ---- STIFFNESS ESTIMATE (2-D frame model, relative units) -------------------
def estimate(rows, cell_w, web, face, x_fixed=False, T=None, L=20.0):
    """Rigid-jointed 2-D frame of the lattice, plates pushed together; returns stiffness per mm width.
    Relative numbers only (E=1). x_fixed=True: faces cannot slide sideways (pad friction)."""
    import numpy as np
    T = T or THICKNESS
    a = cell_w / 2.0
    y0, y1 = (face, T - face) if face > 0 else (0.0, T)
    b = (y1 - y0) / (2 * rows)
    nodes, elems = {}, []
    def nid(k):
        return nodes.setdefault(k, len(nodes))
    imax = int(L / a) + 1
    for r in range(2 * rows + 1):
        for i in range(-imax, imax + 1):
            if (i + r) % 2 == 0 or abs(i * a) > L:
                continue
            for di in (1, -1):
                if r + 1 <= 2 * rows and abs((i + di) * a) <= L:
                    elems.append((nid((i, r)), nid((i + di, r + 1)), web))
    if face > 0:
        for r in (0, 2 * rows):
            xs = sorted(i for (i, rr) in list(nodes) if rr == r)
            elems += [(nid((i, r)), nid((j, r)), face) for i, j in zip(xs, xs[1:])]
    pos = np.zeros((len(nodes), 2))
    for (i, r), k in nodes.items():
        pos[k] = (i * a, y0 + r * b)
    n = len(pos); K = np.zeros((3 * n, 3 * n))
    for p_, q_, t in elems:
        d = pos[q_] - pos[p_]; Le = float(np.hypot(*d)); c, s = d / Le
        k = np.zeros((6, 6)); ea = t / Le; ei = t ** 3 / 12.0
        k[np.ix_([0, 3], [0, 3])] = ea * np.array([[1, -1], [-1, 1]])
        k[np.ix_([1, 2, 4, 5], [1, 2, 4, 5])] += ei / Le ** 3 * np.array(
            [[12, 6 * Le, -12, 6 * Le], [6 * Le, 4 * Le ** 2, -6 * Le, 2 * Le ** 2],
             [-12, -6 * Le, 12, -6 * Le], [6 * Le, 2 * Le ** 2, -6 * Le, 4 * Le ** 2]])
        Rm = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]); Tm = np.zeros((6, 6)); Tm[:3, :3] = Rm; Tm[3:, 3:] = Rm
        dofs = [3 * p_, 3 * p_ + 1, 3 * p_ + 2, 3 * q_, 3 * q_ + 1, 3 * q_ + 2]
        K[np.ix_(dofs, dofs)] += Tm.T @ k @ Tm
    bottom = [k for (i, r), k in nodes.items() if r == 0]
    top = [k for (i, r), k in nodes.items() if r == 2 * rows]
    fixed = {}
    for k in bottom: fixed[3 * k + 1] = 0.0
    for k in top:    fixed[3 * k + 1] = -1.0
    if x_fixed:
        for k in bottom + top: fixed[3 * k] = 0.0
    else:
        fixed[3 * bottom[0]] = 0.0
    fd = np.array(sorted(fixed)); fv = np.array([fixed[d] for d in fd])
    free = np.setdiff1d(np.arange(3 * n), fd)
    u = np.zeros(3 * n); u[fd] = fv
    u[free] = np.linalg.solve(K[np.ix_(free, free)] + 1e-12 * np.eye(len(free)), -K[np.ix_(free, fd)] @ fv)
    F = (K @ u)[[3 * k + 1 for k in top]].sum()
    return abs(F) / (pos[:, 0].max() - pos[:, 0].min())

# ---- DERIVED ----------------------------------------------------------------
R  = DIAMETER / 2.0
ZC = FOOT_H + GAP + R
T_ = THICKNESS
theta = math.radians(SUPPORT_ANGLE)

disc_prof = [(R * math.sin(2 * math.pi * i / CIRCLE_SEG), ZC - R * math.cos(2 * math.pi * i / CIRCLE_SEG))
             for i in range(CIRCLE_SEG)]

def cradle_profile():
    """Foot cross-section (x, z): disc rim lowered by GAP at the bottom, widening to
    GAP_SIDE on the steep flanks, plus a flat shelf and straight outer walls."""
    arc = []
    for i in range(ARC_SEG + 1):
        t = -theta + 2 * theta * i / ARC_SEG
        ramp = min(1.0, max(0.0, (abs(t) - math.radians(40)) / math.radians(SUPPORT_ANGLE - 40)))
        g = GAP + (GAP_SIDE - GAP) * ramp
        arc.append(((R + g) * math.sin(t), ZC - (R + g) * math.cos(t)))
    xe, ze = arc[-1]
    xo = xe + FLARE_X
    # bottom, up the right wall, across the shelf, down the arc (right to left), across the left shelf
    return [(-xo, 0.0), (xo, 0.0), (xo, ze)] + arc[::-1] + [(-xo, ze)]

def wire_profile():
    """Half-stadium outline (x, z): rounded end toward the disc centre, open toward the rim."""
    a = math.radians(WIRE_ANGLE)
    ux, uz = math.sin(a), math.cos(a)                  # unit vector from the disc centre toward the notch
    vx, vz = uz, -ux                                   # perpendicular
    r = WIRE_D / 2.0
    d_c = R - WIRE_DEPTH + r                           # centre of the rounded end, measured from the disc centre
    cx, cz = d_c * ux, ZC + d_c * uz
    pts = []
    for k in range(25):
        t = math.pi * k / 24.0
        pts.append((cx + r * (math.cos(t) * vx - math.sin(t) * ux),
                    cz + r * (math.cos(t) * vz - math.sin(t) * uz)))
    L = (WIRE_DEPTH - r) + 3.0                         # run out 3 mm past the rim
    pts.append((cx - r * vx + L * ux, cz - r * vz + L * uz))
    pts.append((cx + r * vx + L * ux, cz + r * vz + L * uz))
    return pts

# ---- BUILD ONE COOKIE -------------------------------------------------------
XOFF = 0.013     # tiny sideways nudge so lattice lines never coincide with the circle's top/bottom vertex

def lattice_cells(rows, cell_w, web, face):
    a = cell_w / 2.0; d = web / 2.0
    plated = face > 0
    if plated: y0, y1 = face, T_ - face
    else:      y0, y1 = -0.02, T_ + 0.02          # tiny overshoot: nodes never sit exactly on a face
    b = (y1 - y0) / (2 * rows)
    cells = []
    imax = int((R + 2 * a) / a) + 1
    for r in range(2 * rows + 1):
        cy = y0 + r * b
        for i in range(-imax, imax + 1):
            if (i + r) % 2:
                continue
            cx = i * a + XOFF
            if plated and r == 0:
                cells.append(inset_convex([(cx - a, cy), (cx + a, cy), (cx, cy + b)], [0, d, d]))
            elif plated and r == 2 * rows:
                cells.append(inset_convex([(cx + a, cy), (cx - a, cy), (cx, cy - b)], [0, d, d]))
            else:
                cells.append(inset_convex([(cx, cy - b), (cx + a, cy), (cx, cy + b), (cx - a, cy)], [d] * 4))
    return cells, b

def build_cookie(rows, cell_w, web, face, name):
    cells, b = lattice_cells(rows, cell_w, web, face)
    lean = math.degrees(math.atan2(cell_w / 2.0, b))
    zlo, zhi = ZC - R - 1.0, ZC + R + 1.0
    slab = make_object("slab_tmp", lambda bm: prism(bm, [(-R - 1, 0.0), (R + 1, 0.0), (R + 1, T_), (-R - 1, T_)], 'z', zlo, zhi))
    def cells_build(bm):
        for c in cells:
            prism(bm, c, 'z', zlo - 1.0, zhi + 1.0)
    cutters = make_object("cells_tmp", cells_build)
    cyl = make_object("cyl_tmp", lambda bm: prism(bm, disc_prof, 'y', -1.0, T_ + 1.0))
    boolean(slab, cutters, 'DIFFERENCE')
    boolean(slab, cyl, 'INTERSECT')
    extra = []
    if WIRE_HOLE:
        wire = make_object("wire_tmp", lambda bm: prism(bm, wire_profile(), 'y', -1.0, T_ + 1.0))
        boolean(slab, wire, 'DIFFERENCE'); extra.append(wire)
    cookie = evaluate(slab, name)
    for o in [slab, cutters, cyl] + extra: remove(o)
    return cookie, lean

def build_foot(name):
    prof = cradle_profile()
    base = make_object("foot_base_tmp", lambda bm: prism(bm, prof, 'y', 0.0, T_))
    parts = []
    def add(builder, nm):
        o = make_object(nm, builder); parts.append(o); boolean(base, o, 'UNION')
    # base plates beside each face (overlap the cradle by 0.3 mm)
    add(lambda bm: prism(bm, [(-(prof[1][0]), 0.0), (prof[1][0], 0.0), (prof[1][0], FLANGE_T), (-(prof[1][0]), FLANGE_T)],
                         'y', -FLANGE, 0.3), "fl_front_tmp")
    add(lambda bm: prism(bm, [(-(prof[1][0]), 0.0), (prof[1][0], 0.0), (prof[1][0], FLANGE_T), (-(prof[1][0]), FLANGE_T)],
                         'y', T_ - 0.3, T_ + FLANGE), "fl_back_tmp")
    if BRACES:
        yf = -(BRACE_GAP + BRACE_T)                       # outer face of the front brace
        hb = BRACE_H
        add(lambda bm: prism(bm, [(-BRACE_HALF_W, 0.0), (BRACE_HALF_W, 0.0), (BRACE_HALF_W, hb), (-BRACE_HALF_W, hb)],
                             'y', yf, -BRACE_GAP), "br_front_tmp")
        add(lambda bm: prism(bm, [(-BRACE_HALF_W, 0.0), (BRACE_HALF_W, 0.0), (BRACE_HALF_W, hb), (-BRACE_HALF_W, hb)],
                             'y', T_ + BRACE_GAP, T_ + BRACE_GAP + BRACE_T), "br_back_tmp")
        for k, gx in enumerate(GUSSET_XS):
            tri_f = [(yf + 0.3, 0.0), (-FLANGE, 0.0), (yf + 0.3, 0.7 * hb)]            # (y, z) front gusset
            tri_b = [(T_ - yf - 0.3, 0.0), (T_ - yf - 0.3, 0.7 * hb), (T_ + FLANGE, 0.0)]
            add(lambda bm, t=tri_f, x=gx: prism(bm, [(y, z) for y, z in t], 'x', x - GUSSET_T / 2, x + GUSSET_T / 2), f"gf{k}_tmp")
            add(lambda bm, t=tri_b, x=gx: prism(bm, [(y, z) for y, z in t], 'x', x - GUSSET_T / 2, x + GUSSET_T / 2), f"gb{k}_tmp")
    foot = evaluate(base, name + "_foot")
    for o in [base] + parts: remove(o)
    return foot

def build_variant(rows, cell_w, web, face, name):
    cookie, lean = build_cookie(rows, cell_w, web, face, name)
    foot = build_foot(name)
    nm_c, vol_c = stats(cookie); nm_f, vol_f = stats(foot)
    full = (2 * R) * T_ * 2 * R
    # reference = the first printed cookie: 10 mm thick, 1 row of 8 mm diamonds, web 0.9, plates 0.9
    ref = estimate(1, 8.0, 0.9, 0.9, T=10.0); ref_x = estimate(1, 8.0, 0.9, 0.9, x_fixed=True, T=10.0)
    e_free = estimate(rows, cell_w, web, face) / ref
    e_fix = estimate(rows, cell_w, web, face, x_fixed=True) / ref_x
    print(f"[{name}] {rows} row(s) of {cell_w:g} mm diamonds, web {web:g}, plates {face:g}; struts lean {lean:.0f} deg from the press axis")
    print(f"  est. stiffness vs the first printed cookie: {e_free:.2f}x (faces free to slide) / {e_fix:.2f}x (faces held) [relative model, not measured]")
    ok = nm_c == 0 and nm_f == 0 and vol_c > 0 and vol_f > 0
    print(f"  cookie non-manifold {nm_c}, {vol_c:.0f} mm^3 | foot non-manifold {nm_f}, {vol_f:.0f} mm^3 {'OK' if ok else 'CHECK MESH'}")
    return cookie, foot

def export_stl(objs, stl_name):
    base = bpy.data.filepath
    folder = os.path.dirname(base) if base else os.path.expanduser("~")
    path = os.path.join(folder, stl_name)
    bpy.ops.object.select_all(action='DESELECT')
    for o in objs: o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    try:
        bpy.ops.wm.stl_export(filepath=path, export_selected_objects=True)
    except Exception:
        bpy.ops.export_mesh.stl(filepath=path, use_selection=True)
    print(f"  Exported: {path}")

# ---- MAIN -------------------------------------------------------------------
for o in [o for o in bpy.data.objects if o.name.startswith("cookie_")]:
    remove(o)
print(f"Disc {DIAMETER:.0f} x {THICKNESS:.0f} mm standing, total height {ZC + R:.1f}")
if WIRE_HOLE:
    print(f"Wiring notch: {WIRE_D:g} mm wide, {WIRE_DEPTH:g} mm deep, {WIRE_ANGLE:g} deg from the top")
    if abs(WIRE_ANGLE) > 40:
        print("WARNING: a notch this far from the top has an overhanging roof; it may need support.")
    if WIRE_DEPTH < WIRE_D / 2:
        print("WARNING: WIRE_DEPTH is less than half of WIRE_D; the notch is shallower than a semicircle.")
for i, v in enumerate(VARIANTS or [{}]):
    rows = v.get('rows', ROWS); cw = v.get('cell_w', CELL_W); web = v.get('web', WEB_T); face = v.get('face', FACE_T)
    name = f"cookie_d{fmt(DIAMETER)}x{fmt(THICKNESS)}_r{rows}_cw{fmt(cw)}_web{fmt(web)}_f{fmt(face)}"
    cookie, foot = build_variant(rows, cw, web, face, name)
    if EXPORT_STL:
        export_stl([cookie, foot], name + ".stl")
    for o in (cookie, foot):
        o.location.x = i * SPACING
print("=== DONE ===")