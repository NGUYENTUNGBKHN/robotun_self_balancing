"""
Two-wheel self-balancing robot simulation (uses robotun_lib).

Structure:
  - Physics (stiff friction) is integrated with small RK4 steps (integrate_rk4).
  - The controller runs at its own, slower rate (like on the MCU) and holds the
    torque constant between updates, so PID state is never touched inside the ODE.
  - Cascade control: position -> velocity -> tilt angle -> wheel torque.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from math import sin, cos, tanh, pi

from robotun_lib.ode.ode_solver import integrate_rk4
from robotun_lib.control.pid_controller import PIDController


# ----------------------------------------------------------------------------
# Robot parameters
# ----------------------------------------------------------------------------
r = 0.0325         # wheel radius (m)
l = 0.2            # distance from wheel axis to center of mass (m)
M = 0.04           # wheel mass (kg)
m = 0.3            # body mass (kg)
g = 9.8

N = 21             # gearbox ratio 1:21
J_rotor = 3e-7     # motor rotor inertia (kg*m^2), estimate: replace with real value
I = 0.5 * M * r**2 + N**2 * J_rotor     # wheel + reflected rotor inertia

B  = 0.003         # viscous friction of the joint (N*m*s/rad)
Tc = 0.4           # Coulomb / breakaway friction of the gearbox (N*m)
eps = 0.05         # tanh smoothing width (rad/s)
b_roll = 0.0005    # rolling friction of the wheel on the ground

# ----------------------------------------------------------------------------
# Controllers (cascade)
# ----------------------------------------------------------------------------
position_pid = PIDController(k_p=1.0, k_d=0.0, k_i=0.0, target=0.0, out_limit=0.5)
velocity_pid = PIDController(k_p=0.4, k_d=0.0, k_i=0.0, target=0.0, out_limit=0.15)

# out_limit must be well above Tc, otherwise the joint stays locked.
# u_min is ~95% of Tc so the output is pushed right past the breakaway torque.
th_pid = PIDController(k_p=4.0, k_d=0.4, k_i=0.0, target=0.0,
                       out_limit=1.0, u_min=0.38)

FALL_ANGLE = 0.6   # rad (~35 deg): controller gives up and cuts the motors
FALL_STOP = 1.4    # rad (~80 deg): body is on the ground, stop the simulation


def balance_control(th, dth, phi, dphi, dt, mode="full"):
    """
    mode = "off"  : no controller, u = 0 (open loop, the robot just falls)
           "th"   : only the inner tilt loop (th_pid), target fixed at 0
           "full" : cascade position -> velocity -> tilt -> torque
    """
    if mode == "off":
        return 0.0

    if mode == "full":
        x = r * phi        # wheel position along the ground (m)
        v = r * dphi       # wheel linear velocity (m/s)

        # Outer loop: position error -> desired velocity
        velocity_pid.target = position_pid.get_control(x, dt)

        # Middle loop: velocity error -> desired tilt angle
        th_pid.target = velocity_pid.get_control(v, dt)
    else:
        th_pid.target = 0.0

    # Inner loop: tilt error -> wheel torque (gyro rate for the D term).
    # Sign is inverted: positive u drives the wheel forward, which must happen
    # when th > target (leaning forward).
    return -th_pid.get_control(th, dt, rate=dth)


def check_fall(th):
    """Reset all integrators and report True if the robot has fallen."""
    if abs(th) > FALL_ANGLE:
        for p in (position_pid, velocity_pid, th_pid):
            p.reset()
        return True
    return False


# ----------------------------------------------------------------------------
# Physics (no controller inside)
# ----------------------------------------------------------------------------
control = {"u": 0.0}   # torque held constant between controller updates


def derivate(state, step, t, dt):
    """Same signature as robotun_lib's solvers. Torque comes from control["u"]."""
    dth, th, dphi, phi = state
    u = control["u"]
    s, c = sin(th), cos(th)

    w_rel = dphi - dth                              # relative speed body <-> wheel
    tau_f = B * w_rel + Tc * tanh(w_rel / eps)      # joint friction (internal torque)
    u_eff = u - tau_f

    _dphi = (m*r*(l*dth**2*s - g*s*c) - b_roll*dphi + u_eff) / (I + m*r**2*s**2)
    _dth = (g*s - r*_dphi*c) / l
    return [_dth, dth, _dphi, dphi]


def simulate(x0, t_end=10.0, ctrl_dt=0.005, phys_dt=0.00005, push=None, mode="full"):
    """
    push = (time, delta_dth): optional velocity kick on the body.
    mode : "off" | "th" | "full", see balance_control().
    The run stops early once the body hits the ground (|th| > FALL_STOP).
    """
    for p in (position_pid, velocity_pid, th_pid):   # start every run from a clean state
        p.reset()

    steps = int(round(ctrl_dt / phys_dt))
    n = int(round(t_end / ctrl_dt))
    x = list(x0)
    rows = []
    for k in range(n):
        t = k * ctrl_dt
        if push and abs(t - push[0]) < ctrl_dt / 2:
            x[0] += push[1]
        dth, th, dphi, phi = x
        if mode != "off" and check_fall(th):
            control["u"] = 0.0
        else:
            control["u"] = balance_control(th, dth, phi, dphi, ctrl_dt, mode)
        for i in range(steps):
            x = integrate_rk4(x, i, t + i * phys_dt, phys_dt, derivate)
        rows.append([t + ctrl_dt, *x, control["u"]])
        if abs(x[1]) > FALL_STOP:
            break
    return np.array(rows)       # columns: t, dth, th, dphi, phi, u


# ----------------------------------------------------------------------------
if __name__ == "__main__":
    import sys
    headless = "--no-show" in sys.argv

    x0 = [0.0, pi/12, 0.0, 0.0]
    runs = {
        "no PID (u = 0)":    simulate(x0, push=(5.0, 0.6), mode="off"),
        "tilt PID only":     simulate(x0, push=(5.0, 0.6), mode="th"),
        "full cascade PID":  simulate(x0, push=(5.0, 0.6), mode="full"),
    }

    for name, d in runs.items():
        th_d, x_d = d[:, 2], d[:, 4] * r
        state = "fell" if abs(th_d[-1]) > FALL_ANGLE else "balanced"
        print(f"{name:18s}: {state:8s} t_end = {d[-1, 0]:.2f} s, "
              f"max |theta| = {np.degrees(np.max(np.abs(th_d))):.1f} deg, "
              f"final theta = {np.degrees(th_d[-1]):.2f} deg, final x = {x_d[-1]*100:.1f} cm")

    # --- Plots: all modes overlaid ---
    fig1, axs = plt.subplots(3, 1, sharex=True, figsize=(8, 7))
    for name, d in runs.items():
        axs[0].plot(d[:, 0], np.degrees(d[:, 2]), label=name)
        axs[1].plot(d[:, 0], d[:, 4] * r * 100)
        axs[2].plot(d[:, 0], d[:, 5])
    axs[0].set_ylabel("theta (deg)"); axs[0].set_ylim(-30, 60); axs[0].legend()
    axs[1].set_ylabel("x (cm)")
    axs[2].set_ylabel("u (N*m)"); axs[2].set_xlabel("time (s)")
    for a_ in axs:
        a_.grid()
    fig1.tight_layout()
    if headless:
        fig1.savefig("balance_plots.png", dpi=110)

    # Animate the full-cascade run
    data = runs["full cascade PID"]
    times = data[:, 0]
    theta = data[:, 2]
    phi = data[:, 4]
    wheel_x = phi * r

    # --- Animation (subsample to ~25 ms per frame) ---
    sub = 5
    th_a, ph_a, wx_a = theta[::sub], phi[::sub], wheel_x[::sub]

    spot_r = 0.7 * r
    wheel_spot_x = wx_a + spot_r * np.cos(ph_a - pi/2)
    wheel_spot_y = r - spot_r * np.sin(ph_a - pi/2)
    mass_x = wx_a + l * np.cos(th_a - pi/2)
    mass_y = r - l * np.sin(th_a - pi/2)

    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(-0.5, 0.5), ylim=(-0.05, 0.3))
    ax.set_aspect('equal')
    ax.grid()

    line, = ax.plot([], [], 'k-', lw=2)
    wheel = plt.Circle((0.0, r), r, color='black', fill=False, lw=2)
    wheel_spot = plt.Circle((0.0, spot_r), 0.002, color='red')
    mass = plt.Circle((0.0, 0.0), 0.01, color='black')
    ax.add_patch(wheel); ax.add_patch(wheel_spot); ax.add_patch(mass)

    def init():
        return []

    def animate(i):
        wheel.set_center((wx_a[i], r))
        wheel_spot.set_center((wheel_spot_x[i], wheel_spot_y[i]))
        mass.set_center((mass_x[i], mass_y[i]))
        line.set_data([wx_a[i], mass_x[i]], [r, mass_y[i]])
        return [line, wheel, wheel_spot, mass]

    ani = animation.FuncAnimation(fig, animate, np.arange(1, len(th_a)),
                                  interval=25, blit=True, init_func=init)

    if headless:
        fig.savefig("balance_frame.png", dpi=90)
    else:
        plt.show()