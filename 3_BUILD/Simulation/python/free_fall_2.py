import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

from numpy import sin, cos, tanh, pi
from robotun_lib.ode.ode_solver import solve, integrate_rk4


# ------------------------------------------------------------------------
# Robot parameters
#-------------------------------------------------------------------------
r = 0.0325      # Wheel radius (m)
l = 0.2         # Distance from wheel axis to center of mass (m)
M = 0.04        # Wheel mass (kg)        
m = 0.3         # body mass (kg)
g = 9.8

# Friction coefficient due to rotation of the body
b1 = 0.1

# Friction coefficient due to rotation of the wheel
b2 = 0.05

N = 21             # gearbox ratio 1:21
J_rotor = 3e-7     # motor rotor inertia (kg*m^2), estimate: replace with real value
I = 0.5 * M * r**2 + N**2 * J_rotor     # wheel + reflected rotor inertia

B  = 0.003         # viscous friction of the joint (N*m*s/rad)
Tc = 0.4           # Coulomb / breakaway friction of the gearbox (N*m)
eps = 0.05         # tanh smoothing width (rad/s)
b_roll = 0.0005    # rolling friction of the wheel on the ground

def derivate(state, step, t, dt):
    dth, th, dphi, phi = state
    u = 0
    s, c = sin(th), cos(th)

    w_rel = dphi - dth                              # relative speed body <-> wheel
    tau_f = B * w_rel + Tc * tanh(w_rel / eps)      # joint friction (internal torque)
    u_eff = u - tau_f

    _dphi = (m*r*(l*dth**2*s - g*s*c) - b_roll*dphi + u_eff) / (I + m*r**2*s**2)
    _dth = (g*s - r*_dphi*c) / l
    return [_dth, dth, _dphi, dphi]


if __name__ == "__main__":
    times = np.linspace(0, 10, 500)
    solution = solve([0.0, pi / 12, .0, .0], times, integrate_rk4, derivate)
    theta = solution[:, 1]
    phi = solution[:, 3]

    wheel_x = phi * r

    spot_r = 0.7 * r
    wheel_spot_x = wheel_x + spot_r * cos(phi - pi / 2)
    wheel_spot_y = r - spot_r * sin(phi - pi / 2)

    mass_x = wheel_x + l * cos(theta - pi / 2)
    mass_y = r - l * sin(theta - pi / 2)

    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(-1.5, 1.5), ylim=(-0.15, 0.15))
    ax.set_aspect('equal')
    ax.grid()

    line, = ax.plot([], [], 'k-', lw=2)
    wheel = plt.Circle((0.0, r), r, color='black', fill=False, lw=2)
    wheel_spot = plt.Circle((0.0, spot_r), 0.002, color='red')
    mass = plt.Circle((0.0, 0.0), 0.01, color='black')


    def init():
        return []


    def animate(i):
        wheel.set_center((wheel_x[i], r))
        wheel_spot.set_center((wheel_spot_x[i], wheel_spot_y[i]))
        mass.set_center((mass_x[i], mass_y[i]))
        line.set_data([wheel_x[i], mass_x[i]], [r, mass_y[i]])
        patches = [line, ax.add_patch(wheel), ax.add_patch(wheel_spot), ax.add_patch(mass)]
        return patches


    ani = animation.FuncAnimation(fig, animate, np.arange(1, len(solution)),
                                  interval=25, blit=True, init_func=init)

    plt.show()

    # ani.save(f"{__file__[:-3]}.gif", writer='imagemagick', fps=24)