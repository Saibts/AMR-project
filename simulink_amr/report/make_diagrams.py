"""Block diagrams for the report (matplotlib): pipeline, control architecture, mission state machine, navigation loop."""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figs")
os.makedirs(OUT, exist_ok=True)
plt.rcParams["font.family"] = "DejaVu Serif"


def box(ax, x, y, w, h, text, fc="#dbe7f5", ec="#27466e", fs=9, bold=False):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.08", fc=fc, ec=ec, lw=1.3))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs,
            fontweight="bold" if bold else "normal", wrap=True)


def arrow(ax, p, q, text=None, color="#333", rad=0.0, fs=7.5, off=(0, 0.08)):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=12, lw=1.2, color=color,
                                 connectionstyle=f"arc3,rad={rad}"))
    if text:
        ax.text((p[0] + q[0]) / 2 + off[0], (p[1] + q[1]) / 2 + off[1], text, ha="center", va="bottom", fontsize=fs, color=color)


def canvas(w, h):
    fig, ax = plt.subplots(figsize=(w, h))
    ax.set_xlim(0, w); ax.set_ylim(0, h); ax.axis("off")
    return fig, ax


# ---- 1. Tool-chain: CAD -> URDF -> Simscape -> simulation -> results ------------------------------------
fig, ax = canvas(13.6, 3.2)
steps = [("SolidWorks\nAssem2 / chassis /\nW / up (.SLDPRT)", "#f6e3c4"),
         ("SW2URDF export\nscout_mini.urdf +\nSTL meshes", "#f6e3c4"),
         ("smimport\n(Simscape\nMultibody)", "#dbe7f5"),
         ("Edit: 6-DOF chassis,\ntorque-driven wheels,\nsensors", "#dbe7f5"),
         ("Add tyre-ground model,\n3D warehouse, movers,\ncontroller", "#dbe7f5"),
         ("Simulate in Simulink\n(ode23t) and log", "#d9efd9"),
         ("Plots, analysis,\nreport", "#d9efd9")]
x = 0.1
for i, (t, c) in enumerate(steps):
    box(ax, x, 1.0, 1.65, 1.3, t, fc=c, fs=7.5)
    if i < len(steps) - 1:
        arrow(ax, (x + 1.65, 1.65), (x + 1.9, 1.65))
    x += 1.9
ax.text(6.8, 2.9, "Tool-chain: from the SolidWorks model to the simulation results", ha="center", fontsize=11, fontweight="bold")
ax.text(6.8, 0.45, "yellow = CAD side   |   blue = Simulink / Simscape Multibody   |   green = simulation and results", ha="center", fontsize=8)
fig.savefig(os.path.join(OUT, "diag_toolchain.png"), dpi=170, bbox_inches="tight"); plt.close(fig)

# ---- 2. Closed-loop architecture ---------------------------------------------------------------------------
fig, ax = canvas(11, 6.2)
box(ax, 0.2, 2.5, 2.0, 1.4, "Simscape Multibody\nplant\nScout Mini (6-DOF chassis,\n4 wheel joints) +\n3D warehouse + movers", fc="#f6e3c4", bold=True, fs=8.5)
box(ax, 0.2, 0.3, 2.0, 1.2, "Tyre-ground contact\nspring-damper Fz\nCoulomb friction\nmu map (oil, water)", fc="#f6e3c4", fs=8)
box(ax, 3.0, 4.6, 2.2, 1.0, "Sensors\nTransform Sensor (pose, vel)\nwheel joint velocity", fc="#e8dff5", fs=8)
box(ax, 3.0, 3.1, 2.2, 1.1, "Warehouse world\nrack / wall / box /\nworker / AMR / truck\ngeometry at time t", fc="#e8dff5", fs=8)
box(ax, 3.0, 1.6, 2.2, 1.1, "Analytic LiDAR\n72 beams, 8 m, 10 Hz\nray-box / ray-circle", fc="#e8dff5", fs=8)
box(ax, 5.9, 3.0, 2.0, 2.4, "Autonomy\noccupancy grid\n(inflated)\nA* planner\nreplanning\nmission FSM\npure pursuit +\nLiDAR repulsion", fc="#dbe7f5", fs=8)
box(ax, 8.4, 3.8, 2.4, 1.4, "Drive control\nskid-steer kinematics\nwheel-speed PI + FF", fc="#d9efd9", fs=8)
box(ax, 8.4, 2.0, 2.4, 1.5, "Slip management\nslip ratio, torque limit,\ntraction control window,\nfriction estimator", fc="#d9efd9", fs=8)
box(ax, 5.9, 0.7, 2.0, 1.2, "Collision monitor\n(ground truth, not used\nby the controller)\nlog: ctrl, tyre, lidar,\npath, mover", fc="#fde0dc", fs=7.5)
arrow(ax, (2.2, 3.4), (3.0, 4.9))
arrow(ax, (2.2, 3.0), (3.0, 3.6))
arrow(ax, (5.2, 5.1), (5.9, 4.8))
arrow(ax, (5.2, 3.6), (5.9, 3.9))
arrow(ax, (5.2, 2.1), (5.9, 3.3))
arrow(ax, (7.9, 4.5), (8.4, 4.5))
arrow(ax, (9.6, 3.8), (9.6, 3.5), "")
ax.plot([9.6, 9.6, 1.2], [2.0, 0.12, 0.12], color="#333", lw=1.2)
arrow(ax, (1.2, 0.12), (1.2, 0.3))
ax.text(5.0, 0.16, "wheel torques (4 x External Force and Torque blocks)", ha="center", fontsize=7.5, color="#333")
arrow(ax, (1.2, 1.5), (1.2, 2.5), "F, M")
arrow(ax, (2.2, 2.9), (3.0, 2.2))
arrow(ax, (6.9, 3.0), (6.9, 1.9))
ax.text(5.5, 5.95, "Closed-loop simulation architecture (Simulink / Simscape Multibody)", ha="center", fontsize=11, fontweight="bold")
fig.savefig(os.path.join(OUT, "diag_architecture.png"), dpi=170, bbox_inches="tight"); plt.close(fig)

# ---- 3. Mission state machine ---------------------------------------------------------------------------------
fig, ax = canvas(11.6, 3.8)
st = [("1  Drive to\nPICKUP", "#dbe7f5"), ("2  LOAD\n(3 s dwell)", "#fdf0c8"), ("3  Drive to\nDELIVERY", "#dbe7f5"),
      ("4  UNLOAD\n(3 s dwell)", "#fdf0c8"), ("5  DONE\n(stop)", "#d9efd9")]
labs = ["< 0.40 m to\npickup", "dwell\nfinished", "< 0.40 m to\ndelivery", "dwell\nfinished"]
x = 0.2
for i, (t, c) in enumerate(st):
    box(ax, x, 1.5, 1.4, 1.1, t, fc=c, bold=True, fs=8.5)
    if i < 4:
        arrow(ax, (x + 1.4, 2.05), (x + 2.2, 2.05), labs[i], fs=6.5, off=(0, 0.1))
    x += 2.2
box(ax, 0.6, 0.1, 10.4, 0.9, "inside states 1 and 3: LiDAR -> occupancy grid -> A* (replan every 0.5 s or when blocked) -> pure pursuit;\n"
    "stop-before-contact speed limit, reactive repulsion, retreat if closing in and rear clear", fc="#f3f3f3", fs=7.5)
ax.text(5.8, 3.4, "Mission state machine of the AMR", ha="center", fontsize=11, fontweight="bold")
fig.savefig(os.path.join(OUT, "diag_mission_fsm.png"), dpi=170, bbox_inches="tight"); plt.close(fig)

# ---- 4. Control step flowchart --------------------------------------------------------------------------------------
fig, ax = canvas(8, 9.6)
items = [("Every 5 ms: read pose, velocity, wheel speeds (ZOH, Ts = 5 ms)", "#e8dff5"),
         ("World model at time t: rects, circles, mover positions", "#e8dff5"),
         ("LiDAR scan (72 rays, 10 Hz) -> hits and nearest ranges", "#e8dff5"),
         ("Update occupancy grid: static map + dynamic layer (2 s hold) -> inflate 0.48 m", "#dbe7f5"),
         ("Path blocked or 0.5 s elapsed?  yes -> A* replan + line-of-sight smoothing", "#dbe7f5"),
         ("Pure pursuit (look-ahead 0.9 m) -> v_ref, w_ref", "#dbe7f5"),
         ("Safety shaping: LiDAR repulsion, stop-before-contact limit, retreat, mission FSM", "#fdf0c8"),
         ("Friction-aware acceleration limit; skid-steer wheel targets", "#d9efd9"),
         ("Slip ratio per wheel -> traction window, torque limit, mu estimate", "#d9efd9"),
         ("Wheel-speed PI + acceleration feed-forward -> torque per wheel (<= 17.5 N m)", "#d9efd9"),
         ("Tyre-ground model -> forces on the wheels -> Multibody integrates (ode23t)", "#f6e3c4")]
y = 9.0
for t, c in items:
    box(ax, 0.4, y, 7.2, 0.65, t, fc=c, fs=8.5)
    if t is not items[-1][0]:
        arrow(ax, (4.0, y), (4.0, y - 0.25))
    y -= 0.85
ax.text(4.0, 9.85, "What happens in every control step", ha="center", fontsize=11, fontweight="bold")
fig.savefig(os.path.join(OUT, "diag_control_step.png"), dpi=170, bbox_inches="tight"); plt.close(fig)
print("diagrams done")
