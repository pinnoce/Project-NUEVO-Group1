# -*- coding: utf-8 -*-
"""
sim_mission.py -- NUEVO mission visualizer with simulated lidar alignment
=========================================================================
Run:  python sim_mission.py

  SPACE       pause / resume
  -> / D      next step
  <- / A      previous step
  R           restart
  Q / Esc     quit

Three panels:
  Left   -- arena top-down view (robot path, FOV cone)
  Top-right  -- body-frame lidar view (point cloud + fit line)
  Bot-right  -- state info
"""

import math, sys
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Circle, Rectangle
np.random.seed(42)

# ============================================================
# Arena geometry  (mm)
# ============================================================
ARENA_XLIM = (-500, 2100)
ARENA_YLIM = (-300, 3800)

OUTER = [(-300,-100),(1900,-100),(1900,3700),(-300,3700),(-300,-100)]
LANE_DIV = [330, 860, 1390]

SHELF_X0, SHELF_W = -300, 80
PATTY_Y  = (400, 630)
LB_Y     = (200, 430)
RB_Y     = (630, 860)

CONES_XY = [(680,2100),(760,2400),(910,2650),(1060,2350),(870,2000)]
CONE_R   = 45

TL_POS = (200, 20)
SS_POS = (1620, 150)

# Wall segments for ray-casting
# Each entry: ((x0,y0),(x1,y1))
WALL_SEGS = [
    ((-300,-100),(-300,3700)),   # left outer wall
    ((1900,-100),(1900,3700)),   # right outer wall
    ((-300,-100),(1900,-100)),   # bottom wall
    ((-300,3700),(1900,3700)),   # top wall
    ((330,-100),(330,3700)),     # lane div 1
    ((860,-100),(860,3700)),     # lane div 2
    ((1390,-100),(1390,3700)),   # lane div 3
    # Burger-assembly shelf face
    ((-220, 200),(-220, 860)),
    # Drop-off shelf face
    ((1760, 700),(1760,1100)),
]

# ============================================================
# Lidar simulation
# ============================================================
N_RAYS       = 360
LIDAR_NOISE  = 8      # mm Gaussian noise on each return
LIDAR_MIN_MM = 50
LIDAR_MAX_MM = 4500


def _ray_hit(px, py, cdx, cdy, ax, ay, bx, by):
    """Return t along ray to hit segment, or inf."""
    wx = bx - ax;  wy = by - ay
    det = cdx * (-wy) - cdy * (-wx)
    if abs(det) < 1e-9:
        return float('inf')
    rx = ax - px;  ry = ay - py
    t = (rx * (-wy) - ry * (-wx)) / det
    s = (cdx * ry  - cdy * rx)   / det
    if t > 0.01 and 0.0 <= s <= 1.0:
        return t
    return float('inf')


def simulate_lidar(px, py, theta_deg):
    """
    Return (N,2) array of lidar returns in BODY frame.
    Body frame: +x = forward, +y = left.
    """
    r_robot = math.radians(theta_deg)
    cr, sr  = math.cos(r_robot), math.sin(r_robot)
    pts = []
    for i in range(N_RAYS):
        a_world = 2.0 * math.pi * i / N_RAYS
        cdx = math.cos(a_world)
        cdy = math.sin(a_world)
        min_t = LIDAR_MAX_MM
        for seg in WALL_SEGS:
            t = _ray_hit(px, py, cdx, cdy, *seg[0], *seg[1])
            if t < min_t:
                min_t = t
        if min_t >= LIDAR_MAX_MM:
            continue
        dist = min_t + np.random.normal(0, LIDAR_NOISE)
        if dist < LIDAR_MIN_MM:
            continue
        # World-frame hit
        wx = px + dist * cdx
        wy = py + dist * cdy
        # Body-frame conversion: rotate by -theta
        dx =  cr * (wx - px) + sr * (wy - py)   # +x = forward
        dy = -sr * (wx - px) + cr * (wy - py)   # +y = left
        pts.append((dx, dy))
    return np.array(pts, dtype=float) if pts else np.zeros((0, 2))


def fit_wall_line(pts, fov_half_deg, fov_center_deg):
    """
    PCA line fit identical to new_main.py.
    Returns (correction_deg, phi_deg, cx, cy) or None on failure.
    correction_deg = how much robot should turn (CCW+) to align.
    """
    if len(pts) == 0:
        return None
    fov   = math.radians(fov_half_deg)
    center = math.radians(fov_center_deg)  # 0=forward, 90=left, -90=right

    mask = [abs(math.atan2(p[1], p[0]) - center) <= fov
            and LIDAR_MIN_MM < math.hypot(p[0], p[1]) < 4000
            for p in pts]
    sel = pts[mask]
    n   = len(sel)
    if n < 8:
        return None

    mx, my = sel[:,0].mean(), sel[:,1].mean()
    sxx = ((sel[:,0]-mx)**2).mean()
    syy = ((sel[:,1]-my)**2).mean()
    sxy = ((sel[:,0]-mx)*(sel[:,1]-my)).mean()

    tr   = sxx + syy
    disc = math.sqrt(max(0, (tr/2)**2 - (sxx*syy - sxy*sxy)))
    major = tr/2 + disc
    minor = tr/2 - disc
    if major <= 0:
        return None
    thickness = math.sqrt(max(0, minor)) / math.sqrt(major)
    if thickness > 0.40:
        return None    # blobby cloud -- not a wall

    phi = 0.5 * math.atan2(2*sxy, sxx - syy)
    phi = (phi + math.pi/2) % math.pi - math.pi/2   # -> (-90, 90]
    phi_deg = math.degrees(phi)

    if fov_center_deg == 0.0:
        # Forward wall: psi = phi + 90 gives heading correction
        psi = phi_deg + 90.0
        psi = (psi + 90.0) % 180.0 - 90.0
        correction = psi
    else:
        # Side wall: principal axis should be along +x (0 deg)
        correction = phi_deg

    return correction, phi_deg, mx, my


# ============================================================
# Trajectory
# ============================================================

def fwd(x, y, theta_deg, dist):
    r = math.radians(theta_deg)
    return x + dist*math.cos(r), y + dist*math.sin(r)

T0  = 90.0
PP  = fwd(0, 0, T0, 500)         # patty position
PLB = (PP[0], PP[1] - 152)       # left bun
PRB = (PP[0], PP[1] + 152)       # right bun
SH0, SH1, SH2, SH3 = T0+98, T0+97, T0+97, T0+93

def shp(x, y, theta): return fwd(x, y, theta, 170)

# (x, y, theta_deg, label, description, fov_side, duration_s)
# fov_side  None | 'forward' | 'left' | 'right'
#   Non-None  => alignment state: robot starts MISALIGNED and corrects
ALIGN_OFFSET = 12.0    # degrees of initial misalignment shown

TRAJECTORY = [
    (0,0,T0,'IDLE',
     'Lift raises to carry height\nGripper opens to full\nWaiting for BTN_1',None,2.5),
    (0,0,T0+30,'TL_TURN_LEFT','Turn 30 deg left to face traffic light',None,1.0),
    (0,0,T0+30,'TL_WATCH','Watching camera for GREEN traffic light\n(blue LED on)',None,2.5),
    (0,0,T0,'TL_TURN_BACK','Green detected!  Turn back  (0.5s pause)',None,1.0),
    PP+(T0,'MOVING_1','Drive 500 mm forward to patty  (0.5s pause)',None,1.5),
    PP+(SH0,'BUR_0_TURN','Turn left to face patty shelf  (0.5s pause)','forward',1.0),
    PP+(SH0,'BUR_0_ALIGN','Lidar fit line -- align perpendicular to shelf','forward',2.0),
    shp(*PP,SH0)+(SH0,'BUR_0_APPROACH','Drive to shelf standoff (300 mm)','forward',1.2),
    shp(*PP,SH0)+(SH0,'BUR_0_MANIP','PICK PATTY\n  lower->close servo->raise to carry',None,2.0),
    PP+(SH0,'BUR_0_RETREAT','Reverse 200 mm from shelf',None,0.8),
    PP+(T0+180,'BUR_TRAVEL_01_TURN','Turn ~82 deg to backward travel  (0.25s)',None,0.8),
    PLB+(T0+180,'BUR_TRAVEL_01_DRIVE','Drive 152 mm to left bun',None,1.0),
    PLB+(SH1,'BUR_1_TURN','Turn right to face left bun shelf  (0.5s)','forward',1.0),
    PLB+(SH1,'BUR_1_ALIGN','Lidar fit line -- align perpendicular to shelf','forward',2.0),
    shp(*PLB,SH1)+(SH1,'BUR_1_APPROACH','Drive to shelf standoff','forward',1.2),
    shp(*PLB,SH1)+(SH1,'BUR_1_MANIP','PLACE patty on LB\n  lower->open gripper->raise',None,2.0),
    PLB+(SH1,'BUR_1_RETREAT','Reverse 200 mm',None,0.8),
    PLB+(T0,'BUR_TRAVEL_12_TURN','Turn ~98 deg right to face forward  (0.25s)',None,0.8),
    PRB+(T0,'BUR_TRAVEL_12_DRIVE','Drive 304 mm to right bun',None,1.0),
    PRB+(SH2,'BUR_2_TURN','Turn left to face right bun shelf  (0.5s)','forward',1.0),
    PRB+(SH2,'BUR_2_ALIGN','Lidar fit line -- align perpendicular to shelf','forward',2.0),
    shp(*PRB,SH2)+(SH2,'BUR_2_APPROACH','Drive to shelf standoff','forward',1.2),
    shp(*PRB,SH2)+(SH2,'BUR_2_MANIP','PICK right bun\n  lower->close servo (bun)->raise',None,2.0),
    PRB+(SH2,'BUR_2_RETREAT','Reverse 200 mm',None,0.8),
    PRB+(T0+180,'BUR_TRAVEL_23_TURN','Turn ~82 deg left  (0.25s)',None,0.8),
    PLB+(T0+180,'BUR_TRAVEL_23_DRIVE','Drive 304 mm back to stack position',None,1.0),
    PLB+(SH3,'BUR_3_TURN','Turn right to face stack shelf  (0.5s)','forward',1.0),
    PLB+(SH3,'BUR_3_ALIGN','Lidar fit line -- align perpendicular to shelf','forward',2.0),
    shp(*PLB,SH3)+(SH3,'BUR_3_APPROACH','Drive to shelf standoff','forward',1.2),
    shp(*PLB,SH3)+(SH3,'BUR_3_MANIP',
     'STACK + PICK full burger\n  lower to bun+patty -> OPEN\n  lower to 0 -> CLOSE -> raise\n  [Burger assembled!]',None,2.5),
    PLB+(SH3,'BUR_3_RETREAT','Reverse 200 mm  (0.5s pause)',None,1.2),
    PLB+(T0-17,'MOV2_TURN_1','Turn right to exit burger area  (0.5s)',None,0.8),
    PLB+(T0-17,'MOV2_ALIGN_LEFT_1','Align parallel to LEFT wall','left',2.0),
    (60,850,T0-17,'MOV2_DRIVE_1','Drive 500 mm',None,1.2),
    (60,850,T0-17,'MOV2_ALIGN_LEFT_2','Align parallel to LEFT wall  (0.5s)','left',2.0),
    (120,1350,T0-17,'MOV2_DRIVE_2','Drive 500 mm  (0.25s)',None,1.2),
    (120,1350,T0-107,'MOV2_TURN_2','Turn right 90 deg  (0.5s)',None,0.8),
    (120,1350,T0-107,'MOV2_ALIGN_LEFT_3','Align parallel to LEFT wall','left',2.0),
    (1100,1250,T0-107,'MOV2_DRIVE_3','Drive 1000 mm  (0.25s)',None,1.5),
    (1100,1250,T0-197,'MOV2_TURN_3','Turn right 90 deg  (0.5s)',None,0.8),
    (1100,1250,T0-197,'MOV2_ALIGN_LEFT_4','Align parallel to LEFT wall','left',2.0),
    (1020,750,T0-197,'MOV2_DRIVE_4','Drive 500 mm  (0.25s)',None,1.2),
    (1020,750,T0-107,'MOV2_TURN_4','Turn LEFT -- switch to right wall  (0.5s)',None,0.8),
    (1020,750,T0-107,'MOV2_ALIGN_RIGHT_1','Align parallel to RIGHT wall','right',2.0),
    (1510,660,T0-107,'MOV2_DRIVE_5','Drive 500 mm  (0.5s)',None,1.2),
    (1510,660,T0-107,'MOV2_ALIGN_RIGHT_2','Align parallel to RIGHT wall','right',2.0),
    (1510,660,T0-17,'MOV2_TURN_5','Turn left  (0.5s)',None,0.8),
    (1510,660,T0-17,'LAPF_RUN','Leashed APF -- navigating to goal\navoiding cone obstacles',None,1.0),
    (900,1900,T0-17,'LAPF navigating','Virtual target steers around cones',None,1.5),
    (1000,3200,T0-17,'LAPF goal reached','0.5s pause at goal',None,1.2),
    (1000,3200,T0-17,'MOV3_ALIGN_PERP','Lidar fit -- align perpendicular to far wall','forward',2.0),
    (1000,3550,T0-17,'MOV3_APPROACH','Drive to wall standoff (150 mm)  0.25s','forward',1.2),
    (1000,3550,T0-107,'MOV3_TURN','Turn right 90 deg  (0.5s)',None,0.8),
    (1000,3550,T0-107,'MOV3_ALIGN_PARA','Align parallel to LEFT wall','left',2.0),
    (1480,3550,T0-107,'MOV3_DRIVE','Drive 500 mm  -- stop 1s',None,1.5),
    (1480,3550,T0-107,'GENDER_ID',
     'Reading camera: identify person gender\nDefaults to female if no detection',None,2.0),
    (1480,3550,T0-107,'MOV4_ALIGN_PERP','Lidar fit -- align perpendicular to wall','forward',2.0),
    (1700,3550,T0-107,'MOV4_APPROACH','Drive to wall standoff  (0.25s)','forward',1.2),
    (1700,3550,T0-197,'MOV4_TURN','Turn right 90 deg  (0.5s)',None,0.8),
    (1700,3550,T0-197,'MOV4_ALIGN_PARA_1','Align parallel to LEFT wall','left',2.0),
    (1700,3050,T0-197,'MOV4_DRIVE_1','Drive 500 mm  (0.5s)',None,1.2),
    (1700,3050,T0-197,'MOV4_ALIGN_PARA_2','Align parallel to LEFT wall','left',2.0),
    (1700,1080,T0-197,'MOV4_DRIVE_2 [FEMALE]',
     'Gender-dependent drive distance\n  FEMALE: 2000 mm  |  MALE: 2130 mm',None,1.8),
    (1700,1080,T0-107,'DROP_TURN','Turn left to face drop-off shelf  (0.5s)','forward',0.8),
    (1700,1080,T0-107,'DROP_ALIGN','Lidar fit -- align perpendicular to shelf','forward',2.0),
    fwd(1700,1080,T0-107,180)+(T0-107,'DROP_APPROACH','Drive to shelf standoff','forward',1.2),
    fwd(1700,1080,T0-107,180)+(T0-107,'DROP_MANIP',
     'PLACE burger on shelf\n  lower->open gripper->raise',None,2.0),
    (1700,1080,T0-107,'DROP_RETREAT','Reverse 200 mm  (0.5s)',None,1.0),
    (1700,1080,T0+173,'DROP_TURN_BACK','Turn right to face forward',None,0.8),
    (1700,1080,T0+173,'STOP_DRIVE_LOOP',
     'Driving forward watching for stop sign\n(threshold 0.95)',None,1.5),
    (1700,300,T0+173,'STOP SIGN DETECTED',
     'Confidence >= 0.95 -- robot stops!\nWaiting 3 seconds',None,2.5),
    (1700,0,T0+173,'STOP_FINAL_DRIVE','Drive final distance past stop sign',None,1.0),
    (1700,0,T0+173,'DONE','Mission complete!\nAll FSM states executed.',None,3.0),
]

# ============================================================
# Frame / interpolation helpers
# ============================================================

FPS          = 30
STEP_FRAMES  = [max(1, int(s[6]*FPS)) for s in TRAJECTORY]
TOTAL_FRAMES = sum(STEP_FRAMES)
STEP_START   = []
_acc = 0
for nf in STEP_FRAMES:
    STEP_START.append(_acc); _acc += nf


def frame_to_step(f):
    f = max(0, min(f, TOTAL_FRAMES-1))
    lo, hi = 0, len(TRAJECTORY)-1
    while lo < hi:
        mid = (lo+hi+1)//2
        if STEP_START[mid] <= f: lo = mid
        else: hi = mid-1
    nf   = STEP_FRAMES[lo]
    frac = (f - STEP_START[lo]) / max(1, nf-1)
    return lo, min(frac, 1.0)


def wrap_deg(d):
    return (d + 180) % 360 - 180


def pose_at(f):
    si, t = frame_to_step(f)
    step  = TRAJECTORY[si]
    x2, y2, theta2 = step[:3]
    fov_side = step[5]

    if fov_side is not None:
        # Alignment state: start misaligned, rotate to correct heading
        # The misalignment "eases in" over the first half, then stays correct
        misalign_start = theta2 - ALIGN_OFFSET
        # Ease function: fast correction in first 60% of state
        t_ease = min(t / 0.6, 1.0)
        theta   = misalign_start + wrap_deg(theta2 - misalign_start) * t_ease
        return x2, y2, theta

    if si == 0 or t == 0.0:
        return x2, y2, theta2
    x1, y1, theta1 = TRAJECTORY[si-1][:3]
    return (
        x1 + (x2-x1)*t,
        y1 + (y2-y1)*t,
        theta1 + wrap_deg(theta2-theta1)*t,
    )

# ============================================================
# Phase colours
# ============================================================

PHASE_COLORS = {
    'IDLE':'#f59e0b','TL':'#3b82f6','MOV1':'#22c55e','BUR':'#ef4444',
    'MOV2':'#8b5cf6','LAPF':'#0ea5e9','MOV3':'#14b8a6','GENDER':'#14b8a6',
    'MOV4':'#f97316','DROP':'#ec4899','STOP':'#dc2626','DONE':'#4ade80',
}

def phase_color(label):
    for k, v in PHASE_COLORS.items():
        if label.upper().startswith(k): return v
    return '#9ca3af'

# ============================================================
# Figure layout
# ============================================================

fig = plt.figure(figsize=(16, 10), facecolor='#0f172a')
fig.canvas.manager.set_window_title(
    'NUEVO Sim  |  SPACE=pause  ARROWS=step  R=restart  Q=quit')

ax_a  = fig.add_axes([0.01, 0.02, 0.55, 0.96], facecolor='#1e293b')   # arena
ax_l  = fig.add_axes([0.58, 0.44, 0.41, 0.54], facecolor='#0f172a')   # lidar
ax_p  = fig.add_axes([0.58, 0.02, 0.41, 0.40], facecolor='#0f172a')   # panel

for ax in (ax_a, ax_l, ax_p):
    for sp in ax.spines.values(): sp.set_color('#334155')
    ax.tick_params(colors='#64748b', labelsize=7)

ax_a.set_xlim(*ARENA_XLIM); ax_a.set_ylim(*ARENA_YLIM)
ax_a.set_aspect('equal')
ax_a.set_xlabel('x (mm)', color='#64748b', fontsize=8)
ax_a.set_ylabel('y (mm)', color='#64748b', fontsize=8)
ax_a.set_title('NUEVO Mission -- Top-Down Arena', color='#e2e8f0', fontsize=10, pad=5)

ax_l.set_facecolor('#0f172a')
ax_l.set_title('Lidar body-frame view', color='#94a3b8', fontsize=8, pad=4)
ax_l.set_xlabel('body y  (+left)', color='#475569', fontsize=7)
ax_l.set_ylabel('body x  (+fwd)', color='#475569', fontsize=7)
ax_l.set_aspect('equal')

ax_p.set_axis_off()

# ── Static arena elements ─────────────────────────────────

ox=[p[0] for p in OUTER]; oy=[p[1] for p in OUTER]
ax_a.plot(ox, oy, color='#94a3b8', linewidth=2, zorder=2)
for dx in LANE_DIV:
    ax_a.plot([dx,dx],[-100,3700],'--',color='#475569',linewidth=1,zorder=2,alpha=0.6)
for (y0,y1),col,lbl in [(PATTY_Y,'#92400e','Patty'),(LB_Y,'#78350f','L-Bun'),(RB_Y,'#78350f','R-Bun')]:
    ax_a.add_patch(Rectangle((SHELF_X0,y0),SHELF_W+10,y1-y0,
                               facecolor=col,edgecolor='#d97706',linewidth=1.2,zorder=3,alpha=0.85))
    ax_a.text(SHELF_X0+5,(y0+y1)/2,lbl,color='#fde68a',fontsize=6.5,va='center',zorder=4)
for yctr,lbl in [(950,'Female'),(780,'Male')]:
    ax_a.add_patch(Rectangle((1760,yctr-80),120,160,
                               facecolor='#1d4ed8',edgecolor='#93c5fd',linewidth=1.2,zorder=3,alpha=0.85))
    ax_a.text(1763,yctr,lbl,color='#bfdbfe',fontsize=6.5,va='center',zorder=4)
for cx,cy in CONES_XY:
    ax_a.add_patch(Circle((cx,cy),CONE_R,facecolor='#f97316',edgecolor='#fff',linewidth=1,zorder=3))
ax_a.add_patch(Circle(TL_POS,35,facecolor='#15803d',edgecolor='#fff',linewidth=1.5,zorder=3))
ax_a.text(TL_POS[0],TL_POS[1]-68,'TL',color='#bbf7d0',fontsize=6,ha='center',zorder=4)
ax_a.add_patch(Circle(SS_POS,40,facecolor='#dc2626',edgecolor='#fff',linewidth=1.5,zorder=3))
ax_a.text(SS_POS[0],SS_POS[1]-70,'STOP',color='#fca5a5',fontsize=6,ha='center',zorder=4)
ax_a.plot(0,0,'w*',markersize=12,zorder=5,alpha=0.7)
ax_a.text(30,-65,'START',color='#e2e8f0',fontsize=7,zorder=4)

# ── Dynamic arena elements ────────────────────────────────

trail_x, trail_y = [], []
trail_line,  = ax_a.plot([],[],'-',color='#4ade80',linewidth=1.2,alpha=0.45,zorder=5)
robot_circle = Circle((0,0),90,facecolor='#1e3a5f',edgecolor='#93c5fd',linewidth=2,zorder=10)
ax_a.add_patch(robot_circle)
arrow_line,  = ax_a.plot([],[],'-', color='#f0f9ff',linewidth=2.5,zorder=11)
arrow_tip,   = ax_a.plot([],[],'>', color='#f0f9ff',markersize=10,zorder=12)
fov_arena    = Wedge((0,0),600,0,0,facecolor='#22d3ee',alpha=0.15,zorder=6)
ax_a.add_patch(fov_arena)
visited_sc   = ax_a.scatter([],[],s=18,c='#fbbf24',zorder=8,alpha=0.5)

# ── Lidar panel elements ──────────────────────────────────

lidar_sc     = ax_l.scatter([],[],s=5,c='#ef4444',alpha=0.7,zorder=5)
fov_cone_l,  = ax_l.plot([],[],'-',color='#22d3ee',linewidth=1.2,alpha=0.6,zorder=6)
fov_cone_r,  = ax_l.plot([],[],'-',color='#22d3ee',linewidth=1.2,alpha=0.6,zorder=6)
fit_line_l,  = ax_l.plot([],[],'-',color='#f0abfc',linewidth=2,zorder=7)
fit_center_l,= ax_l.plot([],[],'+',color='#f0abfc',markersize=10,zorder=8)
robot_dot_l  = ax_l.scatter([0],[0],s=100,c='#93c5fd',marker='s',zorder=9)
ax_l.axhline(0,color='#334155',linewidth=0.5,alpha=0.5)
ax_l.axvline(0,color='#334155',linewidth=0.5,alpha=0.5)
# Forward arrow in lidar view
ax_l.annotate('fwd',xy=(0,300),xytext=(0,0),
               arrowprops=dict(arrowstyle='->',color='#64748b'),
               color='#64748b',fontsize=7,ha='center',zorder=4)
lidar_info = ax_l.text(0.04,0.97,'',transform=ax_l.transAxes,
                        color='#f0abfc',fontsize=7.5,va='top',fontfamily='monospace',zorder=10)

# ── State panel text ──────────────────────────────────────

txt_state = ax_p.text(0.05,0.97,'',transform=ax_p.transAxes,
                       color='#f1f5f9',fontsize=12,va='top',fontfamily='monospace')
txt_desc  = ax_p.text(0.05,0.78,'',transform=ax_p.transAxes,
                       color='#cbd5e1',fontsize=8.5,va='top',fontfamily='monospace')
txt_stat  = ax_p.text(0.05,0.38,'',transform=ax_p.transAxes,
                       color='#64748b',fontsize=8,va='top',fontfamily='monospace')
txt_keys  = ax_p.text(0.05,0.12,'',transform=ax_p.transAxes,
                       color='#475569',fontsize=7.5,va='top',fontfamily='monospace')
txt_keys.set_text(
    "SPACE      pause / resume\n"
    "-> or D    next step\n"
    "<- or A    prev step\n"
    "R          restart\n"
    "Q / Esc    quit")

pb_ax = fig.add_axes([0.59,0.14,0.39,0.018])
pb_ax.set_xlim(0,1); pb_ax.set_ylim(0,1); pb_ax.set_axis_off()
pb_ax.barh(0.5,1.0,height=1.0,color='#1e293b',left=0,align='center',zorder=0)
pb_fill = pb_ax.barh(0.5,0.0,height=1.0,color='#22c55e',left=0,align='center')[0]

# ============================================================
# Per-frame draw
# ============================================================

def draw_frame(f):
    x, y, theta = pose_at(f)
    si, t_frac  = frame_to_step(f)
    step = TRAJECTORY[si]
    col  = phase_color(step[3])

    # ── Arena: trail ─
    trail_x.append(x); trail_y.append(y)
    if len(trail_x) > 800: trail_x.pop(0); trail_y.pop(0)
    trail_line.set_data(trail_x, trail_y)

    # ── Arena: robot ─
    robot_circle.set_center((x, y))
    robot_circle.set_facecolor(col)

    # ── Arena: heading arrow ─
    r = math.radians(theta)
    tx = x + 140*math.cos(r); ty = y + 140*math.sin(r)
    arrow_line.set_data([x,tx],[y,ty])
    # Rotate the '>' marker with the heading
    arrow_tip.set_data([tx],[ty])
    arrow_tip.set_marker((3, 0, -theta))   # rotated triangle

    # ── Arena: FOV wedge ─
    fov_side = step[5]
    if fov_side:
        ac = theta
        if fov_side == 'left':  ac = theta + 90
        elif fov_side == 'right': ac = theta - 90
        fov_arena.set_visible(True)
        fov_arena.set_center((x, y))
        fov_arena.set_theta1(ac - 20); fov_arena.set_theta2(ac + 20)
        fov_arena.set_facecolor(
            '#22d3ee' if fov_side=='forward' else
            '#a78bfa' if fov_side=='left' else '#fb923c')
    else:
        fov_arena.set_visible(False)

    # ── Arena: visited dots ─
    pts = [(TRAJECTORY[i][0],TRAJECTORY[i][1]) for i in range(si+1)]
    visited_sc.set_offsets(pts)

    # ── Lidar panel ─
    raw = simulate_lidar(x, y, theta)

    if fov_side and len(raw) > 0:
        fov_center_deg = 0.0 if fov_side=='forward' else (90.0 if fov_side=='left' else -90.0)
        fov_half       = 20.0

        # Show in body frame: plot_X = body_y, plot_Y = body_x
        px_all = raw[:,1]; py_all = raw[:,0]

        # Range for lidar axes
        rng  = 1200
        ax_l.set_xlim(-rng, rng); ax_l.set_ylim(-200, rng*1.8)

        lidar_sc.set_offsets(np.column_stack([px_all, py_all]))

        # FOV cone lines (in body frame plot coords)
        fov_r    = math.radians(fov_half)
        ctr_r    = math.radians(fov_center_deg)
        cone_len = rng * 1.5
        for edge_sign, line_obj in ((-1, fov_cone_l),(+1, fov_cone_r)):
            a    = ctr_r + edge_sign * fov_r
            # body_x = cos(a)*cone_len, body_y = sin(a)*cone_len
            # plot: X=body_y, Y=body_x
            line_obj.set_data([0, cone_len*math.sin(a)],
                              [0, cone_len*math.cos(a)])
            line_obj.set_color('#22d3ee' if fov_side=='forward' else
                               '#a78bfa' if fov_side=='left' else '#fb923c')

        # Fit line
        result = fit_wall_line(raw, fov_half, fov_center_deg)
        if result is not None:
            corr, phi_deg, cx, cy = result
            phi_r    = math.radians(phi_deg)
            seg      = 450.0
            # Principal axis direction in body frame: (cos phi, sin phi)
            # plot: X=body_y, Y=body_x
            dx_plot  = math.sin(phi_r) * seg
            dy_plot  = math.cos(phi_r) * seg
            fit_line_l.set_data([cy-dx_plot, cy+dx_plot],
                                [cx-dy_plot, cx+dy_plot])
            fit_center_l.set_data([cy],[cx])

            # How aligned is the robot right now?
            aligned = abs(corr) <= 2.0
            status  = 'ALIGNED' if aligned else f'correction  {corr:+.1f} deg'
            col_txt = '#4ade80' if aligned else '#fbbf24'
            lidar_info.set_text(
                f'phi = {phi_deg:+.1f} deg\n'
                f'correction = {corr:+.1f} deg\n'
                f'{status}')
            lidar_info.set_color(col_txt)
        else:
            fit_line_l.set_data([],[])
            fit_center_l.set_data([],[])
            lidar_info.set_text('fitting...\n(not enough\nwall points)')
            lidar_info.set_color('#94a3b8')

        ax_l.set_title(f'Lidar body-frame  --  {step[3]}',
                       color='#94a3b8', fontsize=8, pad=4)

    elif len(raw) > 0:
        # Show all lidar points even in non-alignment states
        rng = 1500
        ax_l.set_xlim(-rng, rng); ax_l.set_ylim(-300, rng*2)
        lidar_sc.set_offsets(np.column_stack([raw[:,1], raw[:,0]]))
        fov_cone_l.set_data([],[])
        fov_cone_r.set_data([],[])
        fit_line_l.set_data([],[])
        fit_center_l.set_data([],[])
        lidar_info.set_text('(no active alignment)')
        lidar_info.set_color('#475569')
        ax_l.set_title('Lidar body-frame view', color='#64748b', fontsize=8, pad=4)
    else:
        lidar_sc.set_offsets(np.zeros((0,2)))
        lidar_info.set_text('')

    # ── State panel ─
    txt_state.set_text('[  ' + step[3] + '  ]')
    txt_state.set_color(col)
    txt_desc.set_text(step[4])
    txt_stat.set_text(
        'Step {:d} / {:d}\n'
        'Pos  ({:.0f}, {:.0f}) mm\n'
        'Heading  {:.1f} deg\n\n'
        '{}'.format(
            si+1, len(TRAJECTORY),
            x, y, theta,
            '[ PAUSED ]' if state['paused'] else '> RUNNING',
        )
    )
    pb_fill.set_width((si+1)/len(TRAJECTORY))
    pb_fill.set_facecolor(col)


# ============================================================
# Keyboard
# ============================================================

state = {'frame':0, 'paused':False, 'quit':False}

def on_key(event):
    k = (event.key or '').lower()
    if k == ' ':
        state['paused'] = not state['paused']
    elif k in ('right','d'):
        si,_ = frame_to_step(state['frame'])
        if si < len(TRAJECTORY)-1:
            state['frame'] = STEP_START[si+1]
        state['paused'] = True
    elif k in ('left','a'):
        si,_ = frame_to_step(state['frame'])
        target = max(0, si-1)
        state['frame'] = STEP_START[target]
        state['paused'] = True
    elif k == 'r':
        state['frame'] = 0
        trail_x.clear(); trail_y.clear()
        state['paused'] = False
    elif k in ('q','escape'):
        state['quit'] = True
        plt.close(fig)

fig.canvas.mpl_connect('key_press_event', on_key)

# ============================================================
# Main loop
# ============================================================

plt.show(block=False)
fig.canvas.draw()

dt = 1.0 / FPS
while plt.fignum_exists(fig.number):
    draw_frame(state['frame'])
    fig.canvas.draw_idle()
    plt.pause(dt)
    fig.canvas.flush_events()
    if not state['paused'] and not state['quit']:
        state['frame'] += 1
        if state['frame'] >= TOTAL_FRAMES:
            state['frame'] = 0
            trail_x.clear(); trail_y.clear()
