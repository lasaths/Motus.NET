#!/usr/bin/env python3
"""Convert Motus.NET BambooTrussDefaultPlan.json into a Dark studio scene.json."""
from __future__ import annotations

import json
import math
from collections import Counter
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PLAN_PATH = REPO_ROOT / "src" / "Motus.Construction" / "Bamboo" / "BambooTrussDefaultPlan.json"
OUT_DEFAULT = REPO_ROOT / "tools" / "studio" / "out" / "warren_truss_scene.json"

# SiteDefaults.Chosen
CANE = 0.24
GAP = 0.03
NODE_Y = 0.19
CLAMP_Y = 0.09
HEADING = 60.0
FEED_DX = -0.06
FEED_Z = 0.32
FEED_YAW = 0.0
PIT_DEPTH = 0.25
FEED_TOP = 0.12
PLANE_Z = 0.11
HUB_HALF = 0.015
POST_R = 0.007
STRUT_R = 0.007
JAW_CLOSED = 14.0
JAW_OPEN_R = 40.0
JAW_OPEN_L = 45.0
PIT_INSET = 0.012
GROUND_REACH = 0.55
GROUND_HALF_Z = 0.45
EARTH_WALL = 0.03
RACK_SEAT = 0.0005
RACK_PEG_INSET = 0.085
RACK_HALF_ALONG = 0.015
RACK_HALF_ACROSS = 0.004
SEG = (0.030, 0.080, 0.088, 0.088, 0.088, 0.046)
AXES = "YXXXY"
DURATION_SEC = 60.0
MIN_DWELL_SEC = 0.35
FPS = 30
LOCKED_MM = 16.0  # jaws read as locked at or below this opening

PANEL = CANE + 2 * GAP
HEIGHT = PANEL * math.sqrt(3) / 2
HALF_SPAN = PANEL
FEED_X = -PANEL + FEED_DX
STRUT_Y = FEED_TOP + 0.007 + RACK_SEAT
POST_TOP = NODE_Y - HUB_HALF
RACK_PEG_OFFSET = CANE / 2 - RACK_PEG_INSET

NODES = {
    "A": (-PANEL, NODE_Y, PLANE_Z),
    "M": (0.0, NODE_Y, PLANE_Z),
    "D": (PANEL, NODE_Y, PLANE_Z),
    "T1": (-PANEL / 2, NODE_Y + HEIGHT, PLANE_Z),
    "T2": (PANEL / 2, NODE_Y + HEIGHT, PLANE_Z),
}
MEMBERS = (
    ("AM", "A", "M"),
    ("AT1", "A", "T1"),
    ("T1M", "T1", "M"),
    ("MT2", "M", "T2"),
    ("T1T2", "T1", "T2"),
    ("MD", "M", "D"),
    ("T2D", "T2", "D"),
)
MEMBER_IDS = [m[0] for m in MEMBERS]
BUILD_ORDER = ["AT1", "T1M", "AM", "T1T2", "MT2", "MD", "T2D"]

# URDF visuals: (link, name, size xyz, origin xyz, role, jaw_key)
VISUALS = (
    ("mount", "base tube", (0.012, 0.014, 0.016), (0, -0.015, 0), "chassis_backbone", None),
    ("mount", "L-grip body", (0.02, 0.012, 0.026), (0, -0.017, 0), "chassis_plate", None),
    ("mount", "L-grip rail", (0.08, 0.004, 0.004), (0, -0.0095, 0), "sled", None),
    ("link_1", "θ1", (0.046, 0.029, 0.034), (0, 0, 0), "chassis_plate", None),
    ("link_1", "link θ2", (0.012, 0.064, 0.016), (0, -0.04, 0), "chassis_backbone", None),
    ("link_2", "θ2", (0.041, 0.061, 0.04), (0, 0, 0), "chassis_plate", None),
    ("link_2", "link θ3", (0.012, 0.072, 0.016), (0, -0.044, 0), "chassis_backbone", None),
    ("link_3", "θ3", (0.041, 0.061, 0.04), (0, 0, 0), "chassis_plate", None),
    ("link_3", "link θ4", (0.012, 0.072, 0.016), (0, -0.044, 0), "chassis_backbone", None),
    ("link_4", "θ4", (0.041, 0.061, 0.04), (0, 0, 0), "chassis_plate", None),
    ("link_4", "link θ5", (0.012, 0.072, 0.016), (0, -0.044, 0), "chassis_backbone", None),
    ("link_5", "θ5", (0.046, 0.029, 0.034), (0, 0, 0), "chassis_plate", None),
    ("link_5", "neck", (0.012, 0.028, 0.016), (0, -0.024, 0), "chassis_backbone", None),
    ("link_5", "R-grip carriage", (0.018, 0.018, 0.022), (0, -0.024, 0), "chassis_plate", None),
    ("link_5", "R-grip rail", (0.08, 0.004, 0.004), (0, -0.034, 0), "sled", None),
)
JAW_VISUALS = (
    ("L_jaw_m", "L-grip jaw-", (0.0045, 0.03, 0.024), "jaw_static", "L"),
    ("L_jaw_p", "L-grip jaw+", (0.0045, 0.03, 0.024), "jaw_static", "L"),
    ("R_jaw_m", "R-grip jaw-", (0.0045, 0.02, 0.022), "jaw_static", "R"),
    ("R_jaw_p", "R-grip jaw+", (0.0045, 0.02, 0.022), "jaw_static", "R"),
)


def eye():
    return [
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    ]


def mul(a, b):
    c = [0.0] * 16
    for r in range(4):
        for k in range(4):
            s = 0.0
            for j in range(4):
                s += a[r * 4 + j] * b[j * 4 + k]
            c[r * 4 + k] = s
    return c


def inv(m):
    r = [0.0] * 16
    for i in range(3):
        for j in range(3):
            r[i * 4 + j] = m[j * 4 + i]
    for i in range(3):
        r[i * 4 + 3] = -(r[i * 4] * m[3] + r[i * 4 + 1] * m[7] + r[i * 4 + 2] * m[11])
    r[15] = 1.0
    return r


def trans(x, y, z):
    m = eye()
    m[3], m[7], m[11] = x, y, z
    return m


def rot(axis, rad):
    c, s = math.cos(rad), math.sin(rad)
    m = eye()
    if axis == "X":
        m[5], m[6], m[9], m[10] = c, -s, s, c
    elif axis == "Y":
        m[0], m[2], m[8], m[10] = c, s, -s, c
    else:
        m[0], m[1], m[4], m[5] = c, -s, s, c
    return m


def pos(m):
    return (m[3], m[7], m[11])


def col_z(m):
    return (m[2], m[6], m[10])


def rx90():
    # Rx(+90°): (x,y,z) -> (x,-z,y)
    return rot("X", math.pi / 2)


def yup_to_studio_mat(m_row):
    """Viewer Y-up row-major 4x4 -> studio Z-up column-major 4x4 list."""
    z = mul(rx90(), m_row)
    # column-major flatten = transpose then row-major
    out = [0.0] * 16
    for r in range(4):
        for c in range(4):
            out[c * 4 + r] = z[r * 4 + c]
    return [round(v, 8) for v in out]


def yup_to_studio_pt(p):
    x, y, z = p
    return [round(x, 8), round(-z, 8), round(y, 8)]


def yup_to_studio_ax(a):
    x, y, z = a
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    return [round(x / n, 8), round(-z / n, 8), round(y / n, 8)]


def motus_to_studio_mat(m_row):
    """Motus/studio Z-up row-major -> column-major."""
    out = [0.0] * 16
    for r in range(4):
        for c in range(4):
            out[c * 4 + r] = m_row[r * 4 + c]
    return [round(v, 8) for v in out]


def motus_pt(m):
    return [round(m[3], 8), round(m[7], 8), round(m[11], 8)]


def motus_ax(m):
    x, y, z = col_z(m)
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    return [round(x / n, 8), round(y / n, 8), round(z / n, 8)]


def tlr(q_deg):
    c0x, c0y, c0z = 1.0, 0.0, 0.0
    c1x, c1y, c1z = 0.0, 1.0, 0.0
    c2x, c2y, c2z = 0.0, 0.0, 1.0
    px = py = pz = 0.0
    for i in range(5):
        px -= SEG[i] * c1x
        py -= SEG[i] * c1y
        pz -= SEG[i] * c1z
        a = q_deg[i] * math.pi / 180.0
        c, s = math.cos(a), math.sin(a)
        if AXES[i] == "X":
            n1x, n1y, n1z = c * c1x + s * c2x, c * c1y + s * c2y, c * c1z + s * c2z
            n2x, n2y, n2z = -s * c1x + c * c2x, -s * c1y + c * c2y, -s * c1z + c * c2z
            c1x, c1y, c1z = n1x, n1y, n1z
            c2x, c2y, c2z = n2x, n2y, n2z
        else:
            n0x, n0y, n0z = c * c0x - s * c2x, c * c0y - s * c2y, c * c0z - s * c2z
            n2x, n2y, n2z = s * c0x + c * c2x, s * c0y + c * c2y, s * c0z + c * c2z
            c0x, c0y, c0z = n0x, n0y, n0z
            c2x, c2y, c2z = n2x, n2y, n2z
    px -= SEG[5] * c1x
    py -= SEG[5] * c1y
    pz -= SEG[5] * c1z
    return [
        c0x, c1x, c2x, px,
        c0y, c1y, c2y, py,
        c0z, c1z, c2z, pz,
        0.0, 0.0, 0.0, 1.0,
    ]


def link_poses(base_gripper, base_world, q):
    """Viewer Y-up poses for mount, link_1..link_5, tool0."""
    if base_gripper == "R":
        mount = mul(base_world, inv(tlr(q)))
    else:
        mount = list(base_world)
    poses = {"mount": mount}
    t = mount
    offsets = (0.030, 0.080, 0.088, 0.088, 0.088)
    for i, off in enumerate(offsets):
        t = mul(t, trans(0.0, -off, 0.0))
        t = mul(t, rot(AXES[i], q[i] * math.pi / 180.0))
        poses[f"link_{i + 1}"] = t
    tool = mul(t, trans(0.0, -0.046, 0.0))
    poses["tool0"] = tool
    return poses


def l_world(base_gripper, base_world, q):
    return base_world if base_gripper != "R" else mul(base_world, inv(tlr(q)))


def r_world(base_gripper, base_world, q):
    return base_world if base_gripper == "R" else mul(base_world, tlr(q))


def to_motus(viewer_m):
    return mul(rx90(), viewer_m)


def plate_shift(opening_mm):
    return opening_mm / 1000.0 / 2.0 + 0.002


def placement_world(x, y, z, pitch_deg, yaw_deg):
    """Viewer strut placement -> Motus Z-up 4x4 (axis = local Z)."""
    t = trans(x, y, z)
    ry = rot("Y", yaw_deg * math.pi / 180.0)
    rx = rot("X", pitch_deg * math.pi / 180.0)
    return to_motus(mul(t, mul(ry, rx)))


def pitch_yaw_for_axis(dx, dy, dz):
    if dx * dx + dz * dz < 1e-12:
        return (-90.0 if dy >= 0 else 90.0, 0.0)
    pitch = math.degrees(math.atan2(-dy, math.sqrt(dx * dx + dz * dz)))
    yaw = math.degrees(math.atan2(dx, dz))
    return pitch, yaw


def prefer_alt(p0, y0, p1, y1):
    if abs(abs(y1) - 90) < 1 and p1 <= 0 and p0 > 0:
        return True
    if abs(y1) < 1e-9 and abs(p1 + 90) < 1:
        return True
    return False


def along_edge(p, q, length=CANE):
    dx, dy, dz = q[0] - p[0], q[1] - p[1], q[2] - p[2]
    ln = math.sqrt(dx * dx + dy * dy + dz * dz)
    dx, dy, dz = dx / ln, dy / ln, dz / ln
    pitch, yaw = pitch_yaw_for_axis(dx, dy, dz)
    ap, ay = pitch_yaw_for_axis(-dx, -dy, -dz)
    if prefer_alt(pitch, yaw, ap, ay):
        pitch, yaw = ap, ay
    cx = (p[0] + q[0]) / 2
    cy = (p[1] + q[1]) / 2
    cz = (p[2] + q[2]) / 2
    return placement_world(cx, cy, cz, pitch, yaw)


def relay_dir(node, direction):
    # Structure is in a z=const plane: N=+z, U=+x, V=+y
    if direction == "z+":
        return (0.0, 0.0, 1.0)
    if direction == "z-":
        return (0.0, 0.0, -1.0)
    a = float(direction) * math.pi / 180.0
    # u=+x, v=+y
    return (math.cos(a), math.sin(a), 0.0)


def relay_placement(member_id):
    # MEMBER>NODE/DIR
    mid, rest = member_id.split(">", 1)
    node, direction = rest.split("/", 1)
    n = NODES[node]
    d = relay_dir(node, direction)
    ln = CANE + 2 * GAP
    a = n
    b = (n[0] + d[0] * ln, n[1] + d[1] * ln, n[2] + d[2] * ln)
    return mid, along_edge(a, b, CANE)


def store_world():
    return placement_world(FEED_X, STRUT_Y, FEED_Z, 0.0, FEED_YAW)


def box_mesh(size, origin=(0, 0, 0)):
    sx, sy, sz = size
    ox, oy, oz = origin
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    # 8 corners relative to origin
    corners = []
    for dx in (-hx, hx):
        for dy in (-hy, hy):
            for dz in (-hz, hz):
                corners.append((ox + dx, oy + dy, oz + dz))
    # index: 0 --z- y- x-, 1 +z, 2 +y -z, 3 +y +z, 4 +x -y -z, ...
    # Build from faces with consistent outward normals
    faces = (
        ((1, 0, 0), ((hx, -hy, -hz), (hx, hy, -hz), (hx, hy, hz), (hx, -hy, hz))),
        ((-1, 0, 0), ((-hx, -hy, hz), (-hx, hy, hz), (-hx, hy, -hz), (-hx, -hy, -hz))),
        ((0, 1, 0), ((-hx, hy, -hz), (-hx, hy, hz), (hx, hy, hz), (hx, hy, -hz))),
        ((0, -1, 0), ((-hx, -hy, hz), (-hx, -hy, -hz), (hx, -hy, -hz), (hx, -hy, hz))),
        ((0, 0, 1), ((-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz))),
        ((0, 0, -1), ((-hx, hy, -hz), (hx, hy, -hz), (hx, -hy, -hz), (-hx, -hy, -hz))),
    )
    positions, normals = [], []
    for nrm, quad in faces:
        verts = [(ox + x, oy + y, oz + z) for x, y, z in quad]
        tris = (verts[0], verts[1], verts[2], verts[0], verts[2], verts[3])
        for v in tris:
            positions.extend(v)
            normals.extend(nrm)
    # 12 unique edges
    edges_idx = (
        (0, 1), (1, 3), (3, 2), (2, 0),
        (4, 5), (5, 7), (7, 6), (6, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    )
    # corner order for edges: x,y,z bits
    def corner(ix, iy, iz):
        return (ox - hx + ix * sx, oy - hy + iy * sy, oz - hz + iz * sz)
    cmap = {
        0: corner(0, 0, 0),
        1: corner(0, 0, 1),
        2: corner(0, 1, 0),
        3: corner(0, 1, 1),
        4: corner(1, 0, 0),
        5: corner(1, 0, 1),
        6: corner(1, 1, 0),
        7: corner(1, 1, 1),
    }
    edges = []
    for a, b in edges_idx:
        edges.extend(cmap[a])
        edges.extend(cmap[b])
    return {
        "positions": [round(v, 6) for v in positions],
        "normals": [round(v, 5) for v in normals],
        "edges": [round(v, 6) for v in edges],
    }


def sample_state(keys, t):
    if t <= keys[0]["T"]:
        k = keys[0]
        return k["Q"][:], k["GR"], k["GL"], k["Hold"], k["Label"], 0
    last = keys[-1]
    if t >= 1 or t >= last["T"]:
        return last["Q"][:], last["GR"], last["GL"], last["Hold"], last["Label"], len(keys) - 1
    i = 0
    while i < len(keys) - 2 and keys[i + 1]["T"] < t:
        i += 1
    a, b = keys[i], keys[i + 1]
    denom = b["T"] - a["T"]
    u = 1.0 if denom <= 1e-12 else (t - a["T"]) / denom
    e = u * u * (3 - 2 * u)
    q = [a["Q"][k] + (b["Q"][k] - a["Q"][k]) * e for k in range(5)]
    return (
        q,
        a["GR"] + (b["GR"] - a["GR"]) * e,
        a["GL"] + (b["GL"] - a["GL"]) * e,
        b["Hold"],
        b["Label"],
        i + 1,
    )


def cycle_index_at(keys, cycles, t):
    _, _, _, _, _, seg = sample_state(keys, t)
    for i, c in enumerate(cycles):
        if c["FirstKey"] <= seg <= c["LastKey"]:
            return i
    return 0 if seg < cycles[0]["FirstKey"] else len(cycles) - 1


def playback_times(keys):
    """Motion times t in [0,1] at FPS, with official dwells at each label block."""
    times = []
    t = 0.0
    nxt = 0
    dwell = 0.0
    dt = 1.0 / FPS
    # skip keys already at t=0 for the 'reached' pointer
    while nxt < len(keys) and keys[nxt]["T"] < t - 1e-12:
        nxt += 1
    # safety cap: 4 minutes of video
    max_steps = int(4 * 60 * FPS)
    for _ in range(max_steps):
        times.append(t)
        if nxt >= len(keys) and dwell <= 0 and t >= 1 - 1e-12:
            break
        if dwell > 0:
            dwell -= dt
            continue
        if nxt >= len(keys):
            t = 1.0
            continue
        key_t = keys[nxt]["T"]
        target = min(t + min(dt, 0.05) / DURATION_SEC, key_t)
        t = target
        if t >= key_t - 1e-12:
            t = key_t
            block_end = nxt + 1 >= len(keys) or keys[nxt + 1]["Label"] != keys[nxt]["Label"]
            nxt += 1
            dwell = MIN_DWELL_SEC if block_end else 0.0
    if times[-1] < 1 - 1e-12:
        times.append(1.0)
    return times


def is_relay(mid):
    return mid and ">" in mid and not mid.startswith("walk")


def member_of_cycle(mid):
    if not mid or mid == "start" or mid.startswith("walk"):
        return None
    return mid


def find_key(keys, cycle, suffix, pred):
    for i in range(cycle["FirstKey"], min(cycle["LastKey"], len(keys) - 1) + 1):
        if keys[i]["Label"].endswith(suffix) and pred(keys[i]):
            return keys[i]
    return None


def build_cycle_worlds(keys, cycles):
    worlds = []
    placed = []  # list of (name, motus 4x4)
    store = store_world()
    for c in cycles:
        mid = member_of_cycle(c["MemberId"])
        if mid is None:
            worlds.append({"active": None, "rest": store, "held": None, "released": None, "placed": list(placed), "miss": None})
            continue
        rest = store
        parked_i = next((i for i, (n, _) in enumerate(placed) if n.startswith("placed-" + mid + ">")), None)
        if parked_i is None and is_relay(mid):
            parked_i = next((i for i, (n, _) in enumerate(placed) if n.startswith("placed-" + mid.split(">")[0] + ">")), None)
        # Pick from a parked relay of this member (T2D after T2D>A/z+)
        owner = mid.split(">")[0] if is_relay(mid) else mid
        parked_i = next((i for i, (n, _) in enumerate(placed) if n.startswith("placed-" + owner + ">")), parked_i)
        if parked_i is not None and not is_relay(mid):
            rest = placed[parked_i][1]
            placed.pop(parked_i)
        close = find_key(keys, c, ":Close", lambda k: k["Hold"] == 1)
        # Close labels can be "L:Close placed-AT1" — those are walk landings, not cane grasp.
        # Member cycles use ":Close" at the end of "AT1:Close".
        if mid and close is not None and ":Close " in close["Label"]:
            close = find_key(keys, c, mid.split(">")[0] + ":Close" if False else ":Close", lambda k: k["Hold"] == 1 and ":Close " not in k["Label"] and k["Label"].endswith(":Close"))
        open_k = find_key(keys, c, ":Open", lambda k: k["Hold"] == 2 and k["Label"].endswith(":Open"))
        held = released = miss = None
        if close is None:
            miss = "no grasp"
        else:
            jaw = to_motus(r_world(close["Base"], close["BaseWorld"], close["Q"]))
            offset = mul(inv(jaw), rest)
            held = offset
            if open_k is not None:
                released = mul(to_motus(r_world(open_k["Base"], open_k["BaseWorld"], open_k["Q"])), offset)
        worlds.append({"active": mid, "rest": rest, "held": held, "released": released, "placed": list(placed), "miss": miss})
        if released is not None:
            placed.append(("placed-" + mid, released))
    return worlds


def stage_copy(label, built_n):
    """Return (phase slug, idx, display label)."""
    placed = f"{min(built_n, 7)}/7"
    if label == "start:Park":
        return "start", "01", "Start · L clamped on A"
    if label.startswith("walk (land)"):
        target = label.split("→", 1)[-1].strip()
        tid = target.split(":")[0].strip()
        rest = target.split(":", 1)[-1] if ":" in target else ""
        if "Grasp" in rest:
            return f"walk_{tid}_grasp", "05", f"Walk · grasp for {tid}"
        if "Close" in rest and "L:" in rest:
            on = rest.split("Close", 1)[-1].strip()
            return f"walk_{tid}_land", "05", f"Walk · land L on {on or tid}"
        if "swap" in rest:
            return f"walk_{tid}_swap", "05", f"Walk · swap base"
        if "Open" in rest:
            return f"walk_{tid}_open", "05", f"Walk · release trailing grip"
        if "Park" in rest or "Retreat" in rest:
            return f"walk_{tid}_park", "05", f"Walk · park"
        if "off" in rest:
            return f"walk_{tid}_off", "05", f"Walk · lift L clear"
        if "to " in rest:
            return f"walk_{tid}_to", "05", f"Walk · reach {rest.split('to',1)[-1].strip()}"
        return f"walk_{tid}", "05", f"Walk toward {tid}"
    # member or relay
    if ":" not in label:
        return slug(label), "00", label
    mid, action = label.split(":", 1)
    is_rel = is_relay(mid)
    owner = mid.split(">")[0] if is_rel else mid
    idx = f"{BUILD_ORDER.index(owner) + 2:02d}" if owner in BUILD_ORDER else "00"
    if is_rel:
        node = mid.split(">")[1].split("/")[0]
        if "Grasp" in action:
            return f"{mid}_grasp", idx, f"Grasp {owner} for relay at {node}"
        if action == "Close":
            return f"{mid}_close", idx, f"Lock R on {owner}"
        if "Lift" in action:
            return f"{mid}_lift", idx, f"Lift {owner}"
        if "pre-insert" in action:
            return f"{mid}_pre", idx, f"Align {owner} at {node}"
        if "insert" in action:
            return f"{mid}_insert", idx, f"Relay {owner} into {node}"
        if action == "Open":
            return f"{mid}_open", idx, f"Release {owner} at {node}"
        if "Park" in action or "Retreat" in action:
            return f"{mid}_park", idx, f"Park after relay"
        if "via" in action:
            return f"{mid}_via", idx, f"Reach store for {owner}"
        return f"{mid}_{slug(action)}", idx, f"Relay {owner} at {node}"
    if "Grasp" in action:
        return f"{mid}_grasp", idx, f"Grasp {owner}"
    if action == "Close":
        return f"{mid}_close", idx, f"Lock R on {owner}"
    if "Lift" in action:
        return f"{mid}_lift", idx, f"Lift {owner}"
    if "carry" in action:
        return f"{mid}_carry", idx, f"Carry {owner}"
    if "pull out" in action:
        return f"{mid}_pull", idx, f"Pull {owner} from relay"
    if "Place" in action:
        return f"{mid}_place", idx, f"Place {owner}  ·  {placed}"
    if action == "Open":
        return f"{mid}_open", idx, f"Release {owner}"
    if "Retreat" in action or "Park" in action:
        return f"{mid}_park", idx, f"Retreat after {owner}"
    if "via" in action:
        return f"{mid}_via", idx, f"Reach store · {owner}"
    return f"{mid}_{slug(action)}", idx, f"{owner} · {action}"


def slug(s):
    out = []
    for ch in s.lower():
        out.append(ch if ch.isalnum() else "_")
    text = "".join(out)
    while "__" in text:
        text = text.replace("__", "_")
    return text.strip("_") or "phase"


def telemetry(label, hold, g_l, g_r, built_n):
    return f"L {g_l:.0f} mm   R {g_r:.0f} mm   {min(built_n, 7)}/7"


def pads_and_pit():
    # anchors at x=±PANEL, z=PLANE_Z
    min_x, max_x = -PANEL, PANEL
    z = PLANE_Z
    w = -(-min_x - PIT_INSET)
    e = max_x - PIT_INSET
    d = PIT_DEPTH
    pads = (
        ("ground-W", w - GROUND_REACH / 2, z, GROUND_REACH / 2, GROUND_HALF_Z),
        ("ground-E", e + GROUND_REACH / 2, z, GROUND_REACH / 2, GROUND_HALF_Z),
    )
    pits = (
        ("pit-wall-W", w - EARTH_WALL / 2, -d / 2, z, EARTH_WALL / 2, d / 2, GROUND_HALF_Z),
        ("pit-wall-E", e + EARTH_WALL / 2, -d / 2, z, EARTH_WALL / 2, d / 2, GROUND_HALF_Z),
        ("pit-floor", (w + e) / 2, -d - EARTH_WALL / 2, z, (e - w) / 2 + EARTH_WALL, EARTH_WALL / 2, GROUND_HALF_Z),
    )
    return pads, pits


def pegs():
    ax = math.sin(FEED_YAW * math.pi / 180.0)
    az = math.cos(FEED_YAW * math.pi / 180.0)
    o = RACK_PEG_OFFSET
    along = abs(ax) > 0.5
    hx = RACK_HALF_ALONG if along else RACK_HALF_ACROSS
    hz = RACK_HALF_ACROSS if along else RACK_HALF_ALONG
    return (
        ("store-peg-1", FEED_X - ax * o, FEED_Z - az * o, hx, hz, FEED_TOP),
        ("store-peg-2", FEED_X + ax * o, FEED_Z + az * o, hx, hz, FEED_TOP),
    )


def fixtures():
    out = []
    thick = 0.008
    pads, pits = pads_and_pit()
    for name, x, z, hx, hz in pads:
        out.append({
            "type": "box",
            "name": name,
            "role": "ground",
            "center": yup_to_studio_pt((x, -thick / 2, z)),
            "size": [round(2 * hx, 6), round(2 * hz, 6), round(thick, 6)],
        })
    for name, x, y, z, hx, hy, hz in pits:
        out.append({
            "type": "box",
            "name": name,
            "role": "pit",
            "center": yup_to_studio_pt((x, y, z)),
            "size": [round(2 * hx, 6), round(2 * hz, 6), round(2 * hy, 6)],
        })
    for name, x, z, hx, hz, top in pegs():
        out.append({
            "type": "box",
            "name": name,
            "role": "peg",
            "center": yup_to_studio_pt((x, top / 2, z)),
            "size": [round(2 * hx, 6), round(2 * hz, 6), round(top, 6)],
        })
    for node, p in NODES.items():
        out.append({
            "type": "box",
            "name": f"hub-{node}",
            "role": "hub",
            "center": yup_to_studio_pt(p),
            "size": [2 * HUB_HALF, 2 * HUB_HALF, 2 * HUB_HALF],
        })
    for name, x, heading in (("anchor-A", -PANEL, 60.0), ("anchor-D", PANEL, 120.0)):
        out.append({
            "type": "cylinder",
            "name": name,
            "role": "post",
            "center": yup_to_studio_pt((x, POST_TOP / 2, PLANE_Z)),
            "radius": POST_R,
            "length": POST_TOP,
        })
    return out


def build_meshes():
    meshes = []
    for link, name, size, origin, role, jaw in VISUALS:
        mesh = box_mesh(size, origin)
        meshes.append({"link": link, "name": name, "role": role, "jawKey": jaw, **mesh})
    for link, name, size, role, jaw in JAW_VISUALS:
        mesh = box_mesh(size, (0, 0, 0))
        meshes.append({"link": link, "name": name, "role": role, "jawKey": jaw, **mesh})
    return meshes


def next_stock_id(cycles, ci):
    for c in cycles[ci:]:
        mid = member_of_cycle(c["MemberId"])
        if mid:
            return mid
    return None


def built_count(world):
    n = 0
    for name, _ in world["placed"]:
        mid = name[len("placed-"):]
        if not is_relay(mid) and mid in MEMBER_IDS:
            n += 1
    return n


def rod_pose(motus_m, visible=True):
    return {"pt": motus_pt(motus_m), "ax": motus_ax(motus_m), "visible": visible}


def build_scene(plan, source_path=None):
    src = Path(source_path) if source_path else PLAN_PATH
    try:
        source_label = str(src.resolve().relative_to(REPO_ROOT))
    except ValueError:
        source_label = str(src)

    keys = plan["Keys"]
    cycles = plan["Cycles"]
    worlds = build_cycle_worlds(keys, cycles)
    misses = [w["miss"] for w in worlds if w["miss"]]
    times = playback_times(keys)

    # Self-check FK on first key
    k0 = keys[0]
    poses0 = link_poses(k0["Base"], k0["BaseWorld"], k0["Q"])
    rw = r_world(k0["Base"], k0["BaseWorld"], k0["Q"])
    err = max(abs(a - b) for a, b in zip(poses0["tool0"], rw))
    if err > 1e-8:
        raise RuntimeError(f"tool0 != RWorld at start: err={err}")

    unique = []
    timeline = []
    sig_map = {}

    for t in times:
        q, g_r, g_l, hold, label, seg = sample_state(keys, t)
        k = keys[seg]
        base, world_m = k["Base"], k["BaseWorld"]
        ci = cycle_index_at(keys, cycles, t)
        w = worlds[ci]
        poses = link_poses(base, world_m, q)
        # jaw links
        sh_l = plate_shift(g_l)
        sh_r = plate_shift(g_r)
        poses["L_jaw_m"] = mul(poses["mount"], trans(-sh_l, 0.0, 0.0))
        poses["L_jaw_p"] = mul(poses["mount"], trans(sh_l, 0.0, 0.0))
        poses["R_jaw_m"] = mul(poses["link_5"], trans(-sh_r, -0.041, 0.0))
        poses["R_jaw_p"] = mul(poses["link_5"], trans(sh_r, -0.041, 0.0))

        closed = []
        if g_l <= LOCKED_MM:
            closed.append("L")
        if g_r <= LOCKED_MM:
            closed.append("R")

        # canes
        rods = {}
        placed_names = {n[len("placed-"):] for n, _ in w["placed"]}
        for name, m in w["placed"]:
            mid = name[len("placed-"):]
            rods[mid] = rod_pose(m, True)
        active = w["active"]
        if active:
            if hold == 1 and w["held"] is not None:
                jaw = to_motus(r_world(base, world_m, q))
                rods[active] = rod_pose(mul(jaw, w["held"]), True)
            elif hold == 2 and w["released"] is not None:
                rods[active] = rod_pose(w["released"], True)
            elif w["rest"] is not None:
                rods[active] = rod_pose(w["rest"], True)
        else:
            sid = next_stock_id(cycles, ci)
            if sid and sid not in placed_names and w["rest"] is not None:
                rods[sid] = rod_pose(w["rest"], True)

        built = built_count(w)
        if active and not is_relay(active) and active in BUILD_ORDER:
            if hold == 2 or "Place" in label or (label.endswith(":Open") and not label.startswith("walk")):
                built = max(built, BUILD_ORDER.index(active) + 1)
        phase, idx, pretty = stage_copy(label, built)

        links = {name: yup_to_studio_mat(m) for name, m in poses.items() if name != "tool0"}
        fr = {
            "phase": phase,
            "phaseIdx": idx,
            "phaseLabel": pretty,
            "telemetry": telemetry(label, hold, g_l, g_r, built),
            "closed": closed,
            "links": links,
            "jaws": {
                "L": yup_to_studio_mat(poses["mount"]),
                "R": yup_to_studio_mat(poses["tool0"]),
            },
            "rods": rods,
        }
        # compact signature
        sig = (
            phase,
            tuple(closed),
            round(q[0], 3), round(q[1], 3), round(q[2], 3), round(q[3], 3), round(q[4], 3),
            round(g_l, 2), round(g_r, 2),
            hold,
            tuple((n, round(p["pt"][0], 4), round(p["pt"][1], 4), round(p["pt"][2], 4)) for n, p in sorted(rods.items())),
        )
        if sig in sig_map:
            timeline.append(sig_map[sig])
        else:
            sig_map[sig] = len(unique)
            timeline.append(len(unique))
            unique.append(fr)

    # static rods: every member name, parked at store (overwritten live)
    store = store_world()
    static_rods = []
    # include relay id as its own rod name when used
    names = list(MEMBER_IDS)
    for w in worlds:
        if w["active"] and w["active"] not in names:
            names.append(w["active"])
    for mid in names:
        static_rods.append({
            "name": mid,
            "role": "host",
            "point": motus_pt(store),
            "axis": motus_ax(store),
            "length": CANE,
            "diameter": 2 * STRUT_R,
        })

    # cameras
    target = [0.0, -0.14, 0.22]
    camera = {
        "target": target,
        "azDeg": -128,
        "elevDeg": 26,
        "dist": 1.72,
        "fov": 32,
        "shadowZ": 0.0,
        "plan": {"target": [0.0, -0.14, 0.12], "up": [0, 1, 0], "offset": [0, 0, 1.5], "halfH": 0.48},
        "elevation": {"target": target, "up": [0, 0, 1], "offset": [0, 1.7, 0], "halfH": 0.50},
    }

    # storyboard stills: pick named phases if present, else evenly
    wanted = [
        ("start", "01", "Start · L clamped on A"),
        ("AT1_place", "02", "Place AT1 · 1/7"),
        ("AM_place", "04", "Place AM · 3/7"),
        ("walk_T1T2_land", "05", "Walk · land on AT1"),
        ("T2D>A/z+_insert", "08", "Relay T2D at A"),
        ("T2D_place", "09", "Place T2D · 7/7"),
    ]
    by_phase = {}
    for i, fr in enumerate(unique):
        by_phase[fr["phase"]] = i
    story = []
    for phase, idx, lab in wanted:
        if phase in by_phase:
            story.append({"still": by_phase[phase], "idx": idx, "label": lab})
    if len(story) < 6:
        n = len(unique)
        story = []
        for j in range(8):
            still = round(j * (n - 1) / 7) if n > 1 else 0
            fr = unique[still]
            story.append({"still": still, "idx": fr["phaseIdx"], "label": fr["phaseLabel"]})

    mid = len(unique) // 2
    scene = {
        "name": "warren_truss",
        "worldUp": "+Z",
        "fps": FPS,
        "meshes": build_meshes(),
        "fixtures": fixtures(),
        "staticRods": static_rods,
        "jawGroups": [{"label": "L", "keys": ["L"]}, {"label": "R", "keys": ["R"]}],
        "paneLabels": {"top": "PLAN VIEW", "bottom": "ELEVATION"},
        "layoutByMode": {"clean": "single", "bare": "single", "annotated": "multiview"},
        "camera": camera,
        "refPlane": None,
        "timeline": timeline,
        "uniqueFrames": unique,
        "keyIndices": {
            "start": 0,
            "mid": mid,
            "end": len(unique) - 1,
            "storyboard": story,
        },
    }
    meta = {
        "source": source_label,
        "keys": len(keys),
        "cycles": [c["MemberId"] for c in cycles],
        "uniqueFrames": len(unique),
        "timeline": len(timeline),
        "durationSec": round(len(timeline) / FPS, 3),
        "fkErr": err,
        "misses": misses,
        "mapped": {
            "Q": "5 ICD joints, degrees, smoothstep between keys",
            "BaseWorld": "ending-key L or R grip frame, viewer Y-up, converted Rx(+90) to studio +Z",
            "GR/GL": "jaw openings mm; plates at ±(opening/2 + 2 mm)",
            "Hold": "1 held in R, 2 released at jaw-leave pose, 0 on store",
            "canes": "store / captured R-offset / released pose; relay T2D>A/z+ kept until T2D pick",
            "labels": "plan Labels, rewritten as stage titles",
            "fixtures": "U-pit, ground pads, hubs, pegs, A/D posts from SiteDefaults.Chosen",
        },
        "unmapped": [],
    }
    return scene, meta


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--plan", default=str(PLAN_PATH))
    ap.add_argument("--out", default=str(OUT_DEFAULT))
    args = ap.parse_args()
    plan_path = Path(args.plan)
    plan = json.loads(plan_path.read_text())
    scene, meta = build_scene(plan, plan_path)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(scene, separators=(",", ":")))
    print(json.dumps(meta, indent=2))
    print(f"wrote {out} ({out.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
