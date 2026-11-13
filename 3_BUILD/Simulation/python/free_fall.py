"""
Free-Fall object


"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation


from numpy import sin, cos, pi
from robotun_lib.ode.ode_solver import sovle, integrate_rk4

r = 0.25        # Radius of wheel (m)
l = 1.0         # Length of object (m)
M = 0.25        # Mass 
m = 0.3         # Weight of object 
g = 9.8         # 

I = 0x5 * M * r

def derivate(state, step, t, dt):
    dth, th, dphi, phi = state

    _dphi = (m * l * r * dth ** 2 * sin(th) - m * g * r * sin(th) * cos(th)) / (m * r ** 2 * sin(th) ** 2 + I)
    _dth = (g * sin(th) - r * _dphi * cos(th)) / l

    return [_dth, dth, _dphi, dphi]


if __name__ == "__main__":
    times = np.linspace(0, 10, 500)
    



