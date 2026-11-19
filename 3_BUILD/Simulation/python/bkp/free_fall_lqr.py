"""
Self-balancing (two-wheel inverted pendulum) robot simulation.

Model: wheel + body treated as a cart-pole system.
  - x      : wheel contact point position (m)
  - theta  : body tilt from vertical (rad), 0 = upright
  - u      : drive force applied at the wheel contact (N), from wheel torque / r

Equations of motion (point-mass pendulum of mass m at height l on a cart of
mass M, standard cart-pole form):

    (M + m) * x_dd + m*l*theta_dd*cos(theta) - m*l*theta_d**2*sin(theta) = u
    l*theta_dd + x_dd*cos(theta) - g*sin(theta) = 0

A continuous-time LQR controller (designed on the linearized model at
theta = 0) provides the balancing torque. Euler-Lagrange derivation and LQR
design are both standard textbook cart-pole results, adapted to a
wheel-driven body instead of a sliding cart.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib import animation
from scipy.linalg import solve_continuous_are

# ---------------------------------------------------------------- parameters
g = 9.81
M = 0.60      # effective wheel+drivetrain mass (kg)
m = 0.90      # body mass above the wheel axle (kg)
l = 0.12      # height of body center of mass above axle (m)
r = 0.045     # wheel radius (m)
dt = 0.01
t_end = 12.0

# ---------------------------------------------------------- linearized model
# state = [x, x_dot, theta, theta_dot], small-angle about theta = 0
A = np.array([
    [0, 1, 0, 0],
    [0, 0, -m * g / M, 0],
    [0, 0, 0, 1],
    [0, 0, (M + m) * g / (M * l), 0],
])
B = np.array([[0], [1 / M], [0], [-1 / (M * l)]])

Q = np.diag([2.0, 1.0, 40.0, 4.0])   # penalize tilt the most
R = np.array([[0.08]])

P = solve_continuous_are(A, B, Q, R)
K = (np.linalg.inv(R) @ B.T @ P).flatten()
print("LQR gain K =", K.round(3))


# -------------------------------------------------------------- dynamics
def dynamics(state, u):
    x, xd, th, thd = state
    s, c = np.sin(th), np.cos(th)

    # solve the 2x2 linear system for [x_dd, theta_dd]
    A11, A12 = (M + m), m * l * c
    A21, A22 = c, l
    b1 = u + m * l * thd ** 2 * s
    b2 = g * s

    det = A11 * A22 - A12 * A21
    x_dd = (b1 * A22 - A12 * b2) / det
    th_dd = (A11 * b2 - A21 * b1) / det
    return np.array([xd, x_dd, thd, th_dd])


def rk4_step(state, u, dt):
    k1 = dynamics(state, u)
    k2 = dynamics(state + 0.5 * dt * k1, u)
    k3 = dynamics(state + 0.5 * dt * k2, u)
    k4 = dynamics(state + dt * k3, u)
    return state + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


# -------------------------------------------------------------- simulate
n = int(t_end / dt)
t = np.linspace(0, t_end, n)
state = np.array([0.0, 0.0, np.radians(12), 0.0])   # start tipped 12 deg
setpoint = np.array([0.0, 0.0, 0.0, 0.0])

X = np.zeros((n, 4))
U = np.zeros(n)
for i in range(n):
    X[i] = state
    u = float(-K @ (state - setpoint))
    u = np.clip(u, -6.0, 6.0)          # actuator saturation
    U[i] = u
    state = rk4_step(state, u, dt)

x_pos, theta = X[:, 0], X[:, 2]

# ---------------------------------------------------------------- plot response
fig1, axs = plt.subplots(2, 1, figsize=(6, 5), sharex=True)
axs[0].plot(t, np.degrees(theta), color="#D85A30")
axs[0].axhline(0, color="0.7", lw=0.8)
axs[0].set_ylabel("tilt angle (deg)")
axs[0].set_title("Self-balancing robot: LQR step response")
axs[1].plot(t, x_pos, color="#185FA5")
axs[1].set_ylabel("wheel position (m)")
axs[1].set_xlabel("time (s)")
for a in axs:
    a.grid(alpha=0.3)
fig1.tight_layout()
# fig1.savefig("/home/claude/balance_response.png", dpi=150)
plt.close(fig1)


# --------------------------------------------------- robot drawing (CAD-like)
def draw_robot(ax, x, theta):
    """Side-view robot: plate on posts, body, two wheels (front/back offset)."""
    ax.clear()
    ax.set_xlim(x - 0.35, x + 0.35)
    ax.set_ylim(-0.02, 0.45)
    ax.set_aspect("equal")
    ax.axis("off")

    axle_y = r
    # body frame in local (unrotated) coordinates, pivoted at the axle
    s, c = np.sin(theta), np.cos(theta)

    def rot(px, py):
        return x + px * c - py * s, axle_y + px * s + py * c

    # two posts
    post_h = 0.16
    for px in (-0.045, 0.045):
        p0 = rot(px, 0.03)
        p1 = rot(px, 0.03 + post_h)
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color="0.25", lw=2.2, solid_capstyle="round")

    # top plate
    plate_pts = [rot(-0.075, 0.03 + post_h), rot(0.075, 0.03 + post_h),
                 rot(0.075, 0.03 + post_h + 0.012), rot(-0.075, 0.03 + post_h + 0.012)]
    ax.add_patch(mpatches.Polygon(plate_pts, closed=True, fc="#9aa0a8", ec="0.2", lw=1))

    # body / PCB block just above the axle
    body_pts = [rot(-0.05, 0.03), rot(0.05, 0.03), rot(0.05, 0.03 + post_h), rot(-0.05, 0.03 + post_h)]
    ax.add_patch(mpatches.Polygon(body_pts, closed=True, fc="#2e2f33", ec="0.1", lw=1))

    # motor cylinder sitting on the axle
    cyl_pts = [rot(-0.05, -0.01), rot(0.05, -0.01), rot(0.05, 0.03), rot(-0.05, 0.03)]
    ax.add_patch(mpatches.Polygon(cyl_pts, closed=True, fc="#b9bdc3", ec="0.2", lw=1))

    # two wheels, slightly offset fore/aft so both are visible (matches the CAD view)
    for off, alpha in ((-0.02, 1.0), (0.02, 0.55)):
        wheel = plt.Circle((x + off, axle_y), r, fc="#2b2b2b", ec="none", alpha=alpha, zorder=1)
        hub = plt.Circle((x + off, axle_y), r * 0.42, fc="#4a3fb0", ec="none", alpha=alpha, zorder=2)
        ax.add_patch(wheel)
        ax.add_patch(hub)
        spoke = x + off + r * 0.9 * np.cos(theta * 3 + off * 40)
        ax.plot([x + off, spoke], [axle_y, axle_y], color="none")  # placeholder keeps zorder stable

    ax.plot([x - 0.35, x + 0.35], [0, 0], color="0.6", lw=1)   # ground


# ------------------------------------------------------------------ animate
fig2, ax2 = plt.subplots(figsize=(6, 4))
stride = 3  # speed up playback

def animate(frame):
    i = frame * stride
    draw_robot(ax2, x_pos[i], theta[i])
    ax2.set_title(f"t = {t[i]:.2f} s   tilt = {np.degrees(theta[i]):.1f} deg")
    return []

n_frames = n // stride
ani = animation.FuncAnimation(fig2, animate, frames=n_frames, interval=dt * stride * 1000)

plt.show()

# ani.save("/home/claude/self_balance.gif", writer="pillow", fps=int(1 / (dt * stride)))