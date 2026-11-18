"""
Self-balancing (two-wheel inverted pendulum) robot simulation with cascaded PID.

Model: wheel + body as a cart-pole with a rigid box body.
  x     : wheel position (m)
  theta : body tilt from vertical (rad), 0 = upright
  u     : drive force at the wheel contact (N) = wheel torque / r

Equations of motion (J = I + m*l^2, I = box inertia about its COM):

  (M + m)*x_dd + m*l*cos(th)*th_dd = u + m*l*th_d^2*sin(th) - (b2/r^2)*x_d
  m*l*cos(th)*x_dd + J*th_dd       = m*g*l*sin(th) - b1*th_d

Controller (cascade, outer -> inner):
  position PID      : x_ref - x                    -> theta_ref (rad)
  angle P gain      : theta_ref - theta            -> rate_ref  (rad/s)
  angular-rate PID  : rate_ref - theta_dot         -> u (N)
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib import animation

# ------------------------------------------------ parameters (single source)
g = 9.81
M = 0.60          # effective wheel+drivetrain mass (kg)
m = 0.90          # body mass (kg)
r = 0.0325        # wheel radius (m)
dt = 0.01         # integration step = controller sample time (s)
t_end = 12.0      # simulation length (s)
b1 = 0.1          # body pivot friction (N*m*s/rad)
b2 = 0.05         # wheel rotational friction (N*m*s/rad)

# Body geometry: box standing on the axle (used by BOTH dynamics and drawing)
body_w, body_h = 0.055, 0.12
plate_w, plate_t = 0.09, 0.010

l = body_h / 2                              # COM height above axle
I = m * (body_w**2 + body_h**2) / 12.0      # box inertia about its COM
J = I + m * l**2                            # inertia about the axle

# ------------------------------------------------ ground ruler
ruler_start = -0.50   # m, left end of the ruler
ruler_end   =  1.00   # m, right end of the ruler (length = end - start)
ruler_major =  0.10   # m, long tick with label (0.10 -> every 10 cm)
ruler_minor =  0.01   # m, short tick (0.01 -> every 1 cm)
view_half   =  0.35   # m, half width of the camera window around the robot

# ------------------------------------------------ controller parameters
theta0_deg = 12.0     # initial tilt (deg)

# Position setpoint: x_ref = 0 until t_step, then x_ref = x_step
x_step = 0.20         # m, position step
t_step = 3.0          # s, time of the step

# Position PID (outer loop): error [m] -> tilt reference [rad]
pos_kp, pos_ki, pos_kd = 0.18, 0.0, 0.25
theta_ref_max = np.radians(15)      # limit on the tilt reference

# Angle loop (P only): tilt error [rad] -> angular-rate reference [rad/s]
angle_kp = 35.0

# Angular-rate PID (inner loop): rate error [rad/s] -> force u [N]
rate_kp, rate_ki, rate_kd = 4.4, 100.0, 0.0
u_max = 60.0                        # actuator force limit (N)


class PID:
    """PID with anti-windup (integral clamp) and output saturation."""
    def __init__(self, kp, ki, kd, out_lim=None, i_lim=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_lim, self.i_lim = out_lim, i_lim
        self.integral = 0.0
        self.prev_err = None

    def update(self, err, dt):
        self.integral += err * dt
        if self.i_lim is not None:
            self.integral = np.clip(self.integral, -self.i_lim, self.i_lim)
        d = 0.0 if self.prev_err is None else (err - self.prev_err) / dt
        self.prev_err = err
        out = self.kp * err + self.ki * self.integral + self.kd * d
        if self.out_lim is not None:
            out = np.clip(out, -self.out_lim, self.out_lim)
        return out


pos_pid = PID(pos_kp, pos_ki, pos_kd, out_lim=theta_ref_max, i_lim=0.2)
rate_pid = PID(rate_kp, rate_ki, rate_kd, out_lim=u_max, i_lim=1.0)


def controller(t, state):
    """Cascade controller. Returns (u, x_ref, theta_ref)."""
    x, xd, th, thd = state
    x_ref = 0.0 if t < t_step else x_step

    # Outer loop: position error -> desired tilt (lean toward the target)
    theta_ref = pos_pid.update(x_ref - x, dt)

    # Middle loop: tilt error -> desired angular rate
    rate_ref = angle_kp * (theta_ref - th)

    # Inner loop: angular-rate error -> drive force.
    # Sign: driving the wheel forward rotates the body backward, so the
    # actuator has negative gain on th_dd; hence the minus sign.
    u = -rate_pid.update(rate_ref - thd, dt)
    return u, x_ref, theta_ref


# -------------------------------------------------------------- dynamics
def dynamics(state, u=0.0):
    """Return d(state)/dt for state = [x, x_dot, theta, theta_dot]."""
    x, xd, th, thd = state
    s, c = np.sin(th), np.cos(th)

    # Mass matrix of the 2x2 linear system for [x_dd, theta_dd]
    A11, A12 = M + m, m * l * c
    A21, A22 = m * l * c, J

    # Right-hand side: applied force, centrifugal term, wheel friction,
    # gravity torque, pivot friction
    f1 = u + m * l * thd**2 * s - (b2 / r**2) * xd
    f2 = m * g * l * s - b1 * thd

    # Solve by Cramer's rule
    det = A11 * A22 - A12 * A21
    x_dd = (f1 * A22 - A12 * f2) / det
    th_dd = (A11 * f2 - A21 * f1) / det
    return np.array([xd, x_dd, thd, th_dd])


def rk4_step(state, u, dt):
    """One 4th-order Runge-Kutta integration step."""
    k1 = dynamics(state, u)
    k2 = dynamics(state + 0.5 * dt * k1, u)
    k3 = dynamics(state + 0.5 * dt * k2, u)
    k4 = dynamics(state + dt * k3, u)
    return state + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


# -------------------------------------------------------------- simulate
n = int(t_end / dt)
t = np.arange(n) * dt                       # time axis matches the RK4 step exactly
state = np.array([0.0, 0.0, np.radians(theta0_deg), 0.0])

# Tilt angle at which the top of the body touches the ground (from real geometry)
top_h = body_h + plate_t
theta_hit = np.arccos(np.clip(-r / top_h, -1, 1)) if top_h > r else np.pi / 2
theta_hit = min(theta_hit, np.radians(90))

X = np.zeros((n, 4))
U = np.zeros(n)
XREF = np.zeros(n)
THREF = np.zeros(n)
n_valid = n
for i in range(n):
    X[i] = state
    if abs(state[2]) >= theta_hit:          # body hit the ground -> stop simulating
        n_valid = i + 1
        break
    u, x_ref, th_ref = controller(t[i], state)
    U[i], XREF[i], THREF[i] = u, x_ref, th_ref
    state = rk4_step(state, u, dt)

X, t = X[:n_valid], t[:n_valid]
U, XREF, THREF = U[:n_valid], XREF[:n_valid], THREF[:n_valid]
x_pos, theta = X[:, 0], X[:, 2]

# ---------------------------------------------------------------- plot response
fig1, axs = plt.subplots(3, 1, figsize=(7, 7), sharex=True)
axs[0].plot(t, np.degrees(theta), color="#D85A30", label="tilt")
axs[0].plot(t, np.degrees(THREF), "--", color="0.5", label="tilt ref")
axs[0].set_ylabel("tilt angle (deg)")
axs[0].set_title("Wheeled inverted pendulum: cascaded PID")
axs[0].legend(loc="upper right")
axs[1].plot(t, x_pos, color="#185FA5", label="position")
axs[1].plot(t, XREF, "--", color="0.5", label="position ref")
axs[1].set_ylabel("wheel position (m)")
axs[1].legend(loc="lower right")
axs[2].plot(t, U, color="#4a3fb0")
axs[2].set_ylabel("drive force u (N)")
axs[2].set_xlabel("time (s)")
for a in axs:
    a.grid(alpha=0.3)
fig1.tight_layout()
# fig1.savefig("pid_response.png", dpi=150)


# --------------------------------------------------- robot drawing
def draw_robot(ax, x, theta, x_ref):
    """Side view, drawn from the same parameters as the dynamics."""
    ax.clear()
    ax.set_xlim(x - view_half, x + view_half)
    ax.set_ylim(-0.055, 0.40)          # extra room below ground for ruler labels
    ax.set_aspect("equal")
    ax.axis("off")

    axle_y = r
    s, c = np.sin(theta), np.cos(theta)

    def rot(px, py):
        # Rotate a body-frame point about the axle:
        # theta > 0 -> top of the body leans toward +x (same as the dynamics)
        return x + px * c + py * s, axle_y + py * c - px * s

    def poly(pts, **kw):
        ax.add_patch(mpatches.Polygon([rot(*p) for p in pts], closed=True, **kw))

    # Ground line
    ax.plot([x - view_half, x + view_half], [0, 0], color="0.4", lw=1.2)

    # Ruler fixed in world coordinates (does not move with the robot).
    # Only draw the part inside both the view window and [ruler_start, ruler_end].
    lo = max(ruler_start, x - view_half)
    hi = min(ruler_end, x + view_half)
    if hi > lo:
        ratio = max(1, int(round(ruler_major / ruler_minor)))
        for k in range(int(np.ceil(lo / ruler_minor - 1e-9)),
                       int(np.floor(hi / ruler_minor + 1e-9)) + 1):
            gx = k * ruler_minor
            if k % ratio == 0:                       # major tick + label (cm)
                ax.plot([gx, gx], [0, -0.016], color="0.2", lw=1.0)
                ax.text(gx, -0.020, f"{round(gx * 100, 6):g}",
                        ha="center", va="top", fontsize=7, color="0.2")
            else:                                    # minor tick
                ax.plot([gx, gx], [0, -0.008], color="0.5", lw=0.7)
        # End markers of the ruler
        for ex in (ruler_start, ruler_end):
            if x - view_half <= ex <= x + view_half:
                ax.plot([ex, ex], [0, -0.03], color="#D85A30", lw=1.5)
    ax.text(x + view_half - 0.005, -0.050, "cm", ha="right", va="bottom",
            fontsize=7, color="0.4")

    # Position setpoint marker (green flag), drawn only if inside the view
    if x - view_half <= x_ref <= x + view_half:
        ax.plot([x_ref, x_ref], [0, 0.06], color="#2a9d4a", lw=1.5, zorder=2)
        ax.add_patch(mpatches.Polygon([(x_ref, 0.06), (x_ref + 0.02, 0.05),
                                       (x_ref, 0.04)], closed=True,
                                      fc="#2a9d4a", ec="none", zorder=2))

    # Body: box standing on the axle, COM at l = body_h / 2
    y0 = l - body_h / 2
    poly([(-body_w/2, y0), (body_w/2, y0), (body_w/2, y0 + body_h), (-body_w/2, y0 + body_h)],
         fc="#2e2f33", ec="0.1", lw=1, zorder=3)

    # Top plate
    poly([(-plate_w/2, y0 + body_h), (plate_w/2, y0 + body_h),
          (plate_w/2, y0 + body_h + plate_t), (-plate_w/2, y0 + body_h + plate_t)],
         fc="#9aa0a8", ec="0.2", lw=1, zorder=3)

    # Center-of-mass marker (red dot) at height l above the axle
    cx, cy = rot(0, l)
    ax.plot(cx, cy, "o", ms=6, mfc="#D85A30", mec="white", zorder=6)

    # Wheel (both wheels overlap in side view), rolls without slipping: phi = x / r
    phi = x / r
    ax.add_patch(plt.Circle((x, axle_y), r, fc="#2b2b2b", ec="0.1", zorder=4))
    ax.add_patch(plt.Circle((x, axle_y), r * 0.55, fc="#4a3fb0", ec="none", zorder=5))
    for k in range(4):                               # spokes show wheel rotation
        a = -phi + k * np.pi / 2
        ax.plot([x, x + r * 0.55 * np.cos(a)], [axle_y, axle_y + r * 0.55 * np.sin(a)],
                color="white", lw=1.5, zorder=6)
    ax.add_patch(plt.Circle((x, axle_y), 0.006, fc="0.85", ec="none", zorder=7))  # hub


# ------------------------------------------------------------------ animate
fig2, ax2 = plt.subplots(figsize=(6, 4))
stride = 3  # draw every 3rd step to speed up playback

def animate(frame):
    i = min(frame * stride, len(t) - 1)
    draw_robot(ax2, x_pos[i], theta[i], XREF[i])
    ax2.set_title(f"t = {t[i]:.2f} s   tilt = {np.degrees(theta[i]):.1f} deg   "
                  f"x = {x_pos[i] * 100:.1f} cm")
    return []

n_frames = int(np.ceil(len(t) / stride))
ani = animation.FuncAnimation(fig2, animate, frames=n_frames, interval=dt * stride * 1000)

plt.show()

# ani.save("self_balance.gif", writer="pillow", fps=int(1 / (dt * stride)))