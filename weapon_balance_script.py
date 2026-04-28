import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from matplotlib.path import Path


CONFIG_FILE = "weapon_config.json"

state = {
    "R": 45.0,

    "w_p1": np.array([38.97, 22.5]),
    "w_p2": np.array([22.5, 38.97]),
    "w_tip": np.array([76.5, 40.5]),

    "c_p1": np.array([-38.97, -22.5]),
    "c_p2": np.array([-22.5, -38.97]),

    "weapon_arc_r": 40.5,
    "comp_arc_r": 40.5,
    "grid_step": 2.0,
}

active_point = None
drag_radius = 7.0
show_state = False
_loading = False
last_weapon_com = np.array([0.0, 0.0])
last_comp_com = np.array([0.0, 0.0])


# =========================
# JSON save / load
# =========================

def state_to_jsonable():
    return {
        "R": float(state["R"]),
        "w_p1": state["w_p1"].tolist(),
        "w_p2": state["w_p2"].tolist(),
        "w_tip": state["w_tip"].tolist(),
        "c_p1": state["c_p1"].tolist(),
        "c_p2": state["c_p2"].tolist(),
        "weapon_arc_r": float(state["weapon_arc_r"]),
        "comp_arc_r": float(state["comp_arc_r"]),
        "grid_step": float(state["grid_step"]),
    }


def load_jsonable(data):
    state["R"] = float(data["R"])
    state["w_p1"] = np.array(data["w_p1"], dtype=float)
    state["w_p2"] = np.array(data["w_p2"], dtype=float)
    state["w_tip"] = np.array(data["w_tip"], dtype=float)
    state["c_p1"] = np.array(data["c_p1"], dtype=float)
    state["c_p2"] = np.array(data["c_p2"], dtype=float)
    state["weapon_arc_r"] = float(data["weapon_arc_r"])
    state["comp_arc_r"] = float(data["comp_arc_r"])
    state["grid_step"] = float(data.get("grid_step", 2.0))

    state["w_p1"] = project_to_circle(state["w_p1"], state["R"])
    state["w_p2"] = project_to_circle(state["w_p2"], state["R"])
    state["c_p1"] = project_to_circle(state["c_p1"], state["R"])
    state["c_p2"] = project_to_circle(state["c_p2"], state["R"])


def export_config(event=None):
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump(state_to_jsonable(), f, indent=4)
    print(f"Exported: {CONFIG_FILE}")


def load_config(event=None):
    global active_point, _loading

    with open(CONFIG_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)

    load_jsonable(data)

    _loading = True
    radius_slider.set_val(state["R"] * 2)
    weapon_slider.set_val(state["weapon_arc_r"])
    comp_slider.set_val(state["comp_arc_r"])
    grid_slider.set_val(state["grid_step"])
    _loading = False

    active_point = None
    draw()
    print(f"Loaded: {CONFIG_FILE}")


# =========================
# Geometry
# =========================

def project_to_circle(p, R):
    n = np.linalg.norm(p)
    if n < 1e-9:
        return np.array([R, 0.0])
    return p / n * R


def angle_of(p):
    return np.arctan2(p[1], p[0])


def circle_arc(p_start, p_end, R, direction="ccw", num=100):
    a1 = angle_of(p_start)
    a2 = angle_of(p_end)

    if direction == "ccw":
        while a2 < a1:
            a2 += 2 * np.pi
    else:
        while a1 < a2:
            a1 += 2 * np.pi

    angles = np.linspace(a1, a2, num)
    return np.column_stack([R * np.cos(angles), R * np.sin(angles)])


def arc_between_points(p1, p2, r, side=1, num=100):
    p1 = np.asarray(p1, dtype=float)
    p2 = np.asarray(p2, dtype=float)

    chord = p2 - p1
    d = np.linalg.norm(chord)

    if d < 1e-9:
        return np.array([p1])

    if d > 2 * r:
        return np.linspace(p1, p2, num)

    mid = (p1 + p2) / 2
    unit = chord / d
    perp = np.array([-unit[1], unit[0]])

    h = np.sqrt(max(r**2 - (d / 2)**2, 0.0))
    center = mid + side * h * perp

    a1 = np.arctan2(p1[1] - center[1], p1[0] - center[0])
    a2 = np.arctan2(p2[1] - center[1], p2[0] - center[0])

    if side > 0:
        while a2 < a1:
            a2 += 2 * np.pi
    else:
        while a1 < a2:
            a1 += 2 * np.pi

    angles = np.linspace(a1, a2, num)

    return np.column_stack([
        center[0] + r * np.cos(angles),
        center[1] + r * np.sin(angles),
    ])


def build_weapon_outline(s):
    base = circle_arc(s["w_p2"], s["w_p1"], s["R"], direction="ccw")
    line = np.linspace(s["w_p1"], s["w_tip"], 40)
    tooth_arc = arc_between_points(s["w_tip"], s["w_p2"], s["weapon_arc_r"], side=-1)
    return np.vstack([base, line, tooth_arc])


def build_comp_outline(s):
    base = circle_arc(s["c_p2"], s["c_p1"], s["R"], direction="ccw")
    outer_arc = arc_between_points(s["c_p1"], s["c_p2"], s["comp_arc_r"], side=-1)
    return np.vstack([base, outer_arc])


# =========================
# Sampling + physics
# =========================

def sample_inside(outline, step):
    min_x, min_y = outline.min(axis=0) - step
    max_x, max_y = outline.max(axis=0) + step

    xs = np.arange(min_x, max_x + step, step)
    ys = np.arange(min_y, max_y + step, step)

    xx, yy = np.meshgrid(xs, ys)
    pts = np.column_stack([xx.ravel(), yy.ravel()])

    path = Path(outline)
    return pts[path.contains_points(pts)]


def compute_props(points):
    if len(points) == 0:
        return {
            "point_count": 0,
            "Mx": 0.0,
            "My": 0.0,
            "Cx": 0.0,
            "Cy": 0.0,
            "Ix": 0.0,
            "Iy": 0.0,
            "Iz": 0.0,
        }

    x = points[:, 0]
    y = points[:, 1]

    Mx = np.sum(x)
    My = np.sum(y)
    Ix = np.sum(y**2)
    Iy = np.sum(x**2)

    return {
        "point_count": len(points),
        "Mx": Mx,
        "My": My,
        "Cx": Mx / len(points),
        "Cy": My / len(points),
        "Ix": Ix,
        "Iy": Iy,
        "Iz": Ix + Iy,
    }


# =========================
# Shape moving
# =========================

def _rotate_point(p, angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([c * p[0] - s * p[1], s * p[0] + c * p[1]])


def rotate_weapon(angle):
    state["w_p1"] = _rotate_point(state["w_p1"], angle)
    state["w_p2"] = _rotate_point(state["w_p2"], angle)
    state["w_tip"] = _rotate_point(state["w_tip"], angle)


def rotate_comp(angle):
    state["c_p1"] = _rotate_point(state["c_p1"], angle)
    state["c_p2"] = _rotate_point(state["c_p2"], angle)


# =========================
# Formatting
# =========================

def dominance_text(value, positive_name, negative_name, quantity):
    if abs(value) < 1e-9:
        return f"{quantity}: balanced"
    if value > 0:
        return f"{quantity}: dominated by {positive_name}"
    return f"{quantity}: dominated by {negative_name}"


def state_text():
    return (
        "EXACT STATE\n"
        f"R: {state['R']:.3f}\n"
        f"weapon_arc_r: {state['weapon_arc_r']:.3f}\n"
        f"comp_arc_r:   {state['comp_arc_r']:.3f}\n"
        f"grid_step:    {state['grid_step']:.3f}\n\n"
        f"w_p1 : {state['w_p1']}\n"
        f"w_p2 : {state['w_p2']}\n"
        f"w_tip: {state['w_tip']}\n"
        f"c_p1 : {state['c_p1']}\n"
        f"c_p2 : {state['c_p2']}\n"
    )


def build_simple_info(wp, cp):
    iz_diff = wp["Iz"] - cp["Iz"]
    mx_total = wp["Mx"] + cp["Mx"]
    my_total = wp["My"] + cp["My"]

    return (
        "TUNING VALUES\n\n"
        f"Weapon COM: ({wp['Cx']:8.2f}, {wp['Cy']:8.2f})\n"
        f"Comp COM:   ({cp['Cx']:8.2f}, {cp['Cy']:8.2f})\n\n"
        f"WEAPON Iz - COMP Iz:\n"
        f"  {iz_diff:14.2f}\n"
        f"  {dominance_text(iz_diff, 'weapon', 'counterweight', 'Iz')}\n\n"
        f"TOTAL moment x:\n"
        f"  {mx_total:14.2f}\n"
        f"  {dominance_text(mx_total, '+x side', '-x side', 'moment x')}\n\n"
        f"TOTAL moment y:\n"
        f"  {my_total:14.2f}\n"
        f"  {dominance_text(my_total, '+y side', '-y side', 'moment y')}\n\n"
        "Goal:\n"
        "  TOTAL moment x ≈ 0\n"
        "  TOTAL moment y ≈ 0\n"
        "  weapon Iz should usually dominate\n"
    )


# =========================
# GUI setup
# =========================

fig = plt.figure(figsize=(15, 9))

ax_text = fig.add_axes([0.02, 0.18, 0.30, 0.78])
ax = fig.add_axes([0.35, 0.18, 0.62, 0.78])

ax_radius = fig.add_axes([0.40, 0.115, 0.40, 0.025])
ax_weapon_r = fig.add_axes([0.40, 0.080, 0.40, 0.025])
ax_comp_r = fig.add_axes([0.40, 0.045, 0.40, 0.025])
ax_grid = fig.add_axes([0.40, 0.010, 0.40, 0.025])

ax_fine_radius = fig.add_axes([0.900, 0.115, 0.055, 0.025])
ax_fine_weapon = fig.add_axes([0.900, 0.080, 0.055, 0.025])
ax_fine_comp   = fig.add_axes([0.900, 0.045, 0.055, 0.025])
ax_fine_grid   = fig.add_axes([0.900, 0.010, 0.055, 0.025])

ax_export = fig.add_axes([0.04, 0.08, 0.12, 0.04])
ax_load = fig.add_axes([0.18, 0.08, 0.12, 0.04])
ax_state_toggle = fig.add_axes([0.04, 0.025, 0.26, 0.04])

radius_slider = Slider(ax_radius, "Base diameter D", 20, 300, valinit=state["R"] * 2, color="tab:blue")
weapon_slider = Slider(ax_weapon_r, "Weapon arc radius", 5, 200, valinit=state["weapon_arc_r"], color="tab:red")
comp_slider = Slider(ax_comp_r, "Comp arc radius", 5, 200, valinit=state["comp_arc_r"], color="tab:green")
grid_slider = Slider(ax_grid, "Grid step", 0.5, 8.0, valinit=state["grid_step"], color="tab:purple")

fine_radius_button = Button(ax_fine_radius, "Fine", color="lightyellow")
fine_weapon_button = Button(ax_fine_weapon, "Fine", color="lightyellow")
fine_comp_button   = Button(ax_fine_comp,   "Fine", color="lightyellow")
fine_grid_button   = Button(ax_fine_grid,   "Fine", color="lightyellow")

export_button = Button(ax_export, "Export JSON", color="lightsteelblue")
load_button = Button(ax_load, "Load JSON", color="lightgreen")
state_button = Button(ax_state_toggle, "Show / Hide Exact State", color="lightgray")

_SLIDER_ORIG_RANGES = {
    "radius": (20.0, 300.0),
    "weapon_r": (5.0, 200.0),
    "comp_r": (5.0, 200.0),
    "grid": (0.5, 8.0),
}
_fine_active = {k: False for k in _SLIDER_ORIG_RANGES}
FINE_FACTOR = 10.0


def _set_slider_range(slider, new_min, new_max):
    cur = np.clip(slider.val, new_min, new_max)
    slider.valmin = new_min
    slider.valmax = new_max
    slider.ax.set_xlim(new_min, new_max)
    slider.set_val(cur)
    fig.canvas.draw_idle()


def _make_fine_toggle(key, slider, button):
    def toggle(event):
        orig_min, orig_max = _SLIDER_ORIG_RANGES[key]
        _fine_active[key] = not _fine_active[key]
        if _fine_active[key]:
            span = (orig_max - orig_min) / FINE_FACTOR
            cur = slider.val
            new_min = max(orig_min, cur - span / 2)
            new_max = min(orig_max, cur + span / 2)
            # ensure full span even when near edges
            if new_max - new_min < span:
                if new_min == orig_min:
                    new_max = min(orig_max, new_min + span)
                else:
                    new_min = max(orig_min, new_max - span)
            button.label.set_text("Coarse")
            button.color = "lightcyan"
        else:
            new_min, new_max = orig_min, orig_max
            button.label.set_text("Fine")
            button.color = "lightyellow"
        _set_slider_range(slider, new_min, new_max)
    return toggle


# =========================
# Draw
# =========================

def draw_grid_square(ax, origin, step):
    x0, y0 = origin
    s = step

    square = np.array([
        [x0, y0],
        [x0 + s, y0],
        [x0 + s, y0 + s],
        [x0, y0 + s],
        [x0, y0],
    ])

    ax.plot(square[:, 0], square[:, 1], linewidth=2, color="tab:purple")
    ax.text(x0, y0 - s * 0.7, f"grid cell {s:.2f} x {s:.2f}", fontsize=8)


def draw():
    global last_weapon_com, last_comp_com

    ax.clear()
    ax_text.clear()

    old_R = state["R"]
    new_R = radius_slider.val / 2

    state["R"] = new_R
    state["weapon_arc_r"] = weapon_slider.val
    state["comp_arc_r"] = comp_slider.val
    state["grid_step"] = grid_slider.val

    if abs(new_R - old_R) > 1e-9:
        state["w_p1"] = project_to_circle(state["w_p1"], new_R)
        state["w_p2"] = project_to_circle(state["w_p2"], new_R)
        state["c_p1"] = project_to_circle(state["c_p1"], new_R)
        state["c_p2"] = project_to_circle(state["c_p2"], new_R)

    R = state["R"]
    step = state["grid_step"]

    weapon_outline = build_weapon_outline(state)
    comp_outline = build_comp_outline(state)

    weapon_points = sample_inside(weapon_outline, step)
    comp_points = sample_inside(comp_outline, step)

    weapon_props = compute_props(weapon_points)
    comp_props = compute_props(comp_points)

    last_weapon_com = np.array([weapon_props["Cx"], weapon_props["Cy"]])
    last_comp_com = np.array([comp_props["Cx"], comp_props["Cy"]])

    circle = plt.Circle((0, 0), R, fill=False, linewidth=2)
    ax.add_patch(circle)

    ax.plot(weapon_outline[:, 0], weapon_outline[:, 1], linewidth=2.5, color="tab:red", label="weapon")
    ax.plot(comp_outline[:, 0], comp_outline[:, 1], linewidth=2.5, color="tab:green", label="counterweight")

    if len(weapon_points):
        ax.scatter(weapon_points[:, 0], weapon_points[:, 1], s=4, alpha=0.25, color="tab:red")
    if len(comp_points):
        ax.scatter(comp_points[:, 0], comp_points[:, 1], s=4, alpha=0.25, color="tab:green")

    for name in ["w_p1", "w_p2", "w_tip"]:
        p = state[name]
        ax.scatter(p[0], p[1], s=100, zorder=5, color="tab:red")
        ax.text(p[0] + 2, p[1] + 2, name, fontsize=9)

    for name in ["c_p1", "c_p2"]:
        p = state[name]
        ax.scatter(p[0], p[1], s=100, zorder=5, color="tab:green")
        ax.text(p[0] + 2, p[1] + 2, name, fontsize=9)

    ax.scatter(last_weapon_com[0], last_weapon_com[1], s=180, marker="x", linewidths=3, color="darkred", zorder=7)
    ax.text(last_weapon_com[0] + 2, last_weapon_com[1] + 2, "weapon COM", fontsize=9, color="darkred")

    ax.scatter(last_comp_com[0], last_comp_com[1], s=180, marker="x", linewidths=3, color="darkgreen", zorder=7)
    ax.text(last_comp_com[0] + 2, last_comp_com[1] + 2, "comp COM", fontsize=9, color="darkgreen")

    ax.scatter(0, 0, s=60, marker="+", color="black")
    ax.axhline(0, linewidth=0.8, color="black")
    ax.axvline(0, linewidth=0.8, color="black")

    draw_grid_square(ax, origin=(-R * 1.45, -R * 1.45), step=step)

    all_values = np.vstack([weapon_outline, comp_outline, np.array([[0, 0]])])
    limit = max(R * 1.8, np.max(np.abs(all_values)) * 1.15)

    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right")
    ax.set_title("Melty Brain Weapon / Counterweight Balance GUI")

    ax_text.axis("off")

    ax_text.text(
        0.0,
        1.0,
        build_simple_info(weapon_props, comp_props),
        va="top",
        ha="left",
        family="monospace",
        fontsize=10,
        fontweight="bold",
    )

    if show_state:
        ax_text.text(
            0.0,
            0.42,
            state_text(),
            va="top",
            ha="left",
            family="monospace",
            fontsize=7.5,
        )

    fig.canvas.draw_idle()


# =========================
# Mouse interaction
# =========================

def find_closest_point(mouse_xy):
    candidates = {
        "w_p1": state["w_p1"],
        "w_p2": state["w_p2"],
        "w_tip": state["w_tip"],
        "c_p1": state["c_p1"],
        "c_p2": state["c_p2"],
        "weapon_com": last_weapon_com,
        "comp_com": last_comp_com,
    }

    best_name = None
    best_dist = float("inf")

    for name, p in candidates.items():
        d = np.linalg.norm(mouse_xy - p)
        if d < best_dist:
            best_dist = d
            best_name = name

    if best_dist <= drag_radius:
        return best_name

    return None


last_mouse_xy = None


def on_press(event):
    global active_point, last_mouse_xy

    if event.inaxes != ax or event.xdata is None or event.ydata is None:
        return

    mouse_xy = np.array([event.xdata, event.ydata])
    active_point = find_closest_point(mouse_xy)
    last_mouse_xy = mouse_xy


def on_motion(event):
    global active_point, last_mouse_xy

    if active_point is None:
        return

    if event.inaxes != ax or event.xdata is None or event.ydata is None:
        return

    mouse_xy = np.array([event.xdata, event.ydata])

    if active_point == "weapon_com":
        angle = angle_of(mouse_xy) - angle_of(last_mouse_xy)
        rotate_weapon(angle)

    elif active_point == "comp_com":
        angle = angle_of(mouse_xy) - angle_of(last_mouse_xy)
        rotate_comp(angle)

    elif active_point in ["w_p1", "w_p2", "c_p1", "c_p2"]:
        state[active_point] = project_to_circle(mouse_xy, state["R"])

    elif active_point == "w_tip":
        state[active_point] = mouse_xy

    last_mouse_xy = mouse_xy
    draw()


def on_release(event):
    global active_point, last_mouse_xy
    active_point = None
    last_mouse_xy = None


def on_slider_change(value):
    if not _loading:
        draw()


def toggle_state(event=None):
    global show_state
    show_state = not show_state
    draw()


# =========================
# Run
# =========================

radius_slider.on_changed(on_slider_change)
weapon_slider.on_changed(on_slider_change)
comp_slider.on_changed(on_slider_change)
grid_slider.on_changed(on_slider_change)

fine_radius_button.on_clicked(_make_fine_toggle("radius", radius_slider, fine_radius_button))
fine_weapon_button.on_clicked(_make_fine_toggle("weapon_r", weapon_slider, fine_weapon_button))
fine_comp_button.on_clicked(_make_fine_toggle("comp_r", comp_slider, fine_comp_button))
fine_grid_button.on_clicked(_make_fine_toggle("grid", grid_slider, fine_grid_button))

export_button.on_clicked(export_config)
load_button.on_clicked(load_config)
state_button.on_clicked(toggle_state)

fig.canvas.mpl_connect("button_press_event", on_press)
fig.canvas.mpl_connect("motion_notify_event", on_motion)
fig.canvas.mpl_connect("button_release_event", on_release)

draw()
plt.show()