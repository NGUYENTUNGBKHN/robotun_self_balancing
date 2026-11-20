import math
import numpy as np

class InvertedPendulum:
    """
    """

    def __init__(self, body_mass, wheel_mass_total, com_height, body_pitch_inertia,
                wheel_radius, wheel_inertia_each, motor_rotor_inertia, gear_ratio,
                longitudinal_viscous=0.0, longitudinal_coulomb=0.0,
                pitch_viscous=0.0):
        required = [body_mass, wheel_mass_total, com_height, body_pitch_inertia,
                    wheel_radius, wheel_inertia_each, motor_rotor_inertia, gear_ratio]
        if any(v is None for v in required):
            raise ValueError("Mechanical parameters are incomplete; fill config/robot.json.")
        
        self.m = body_mass
        self.M_physical = wheel_mass_total
        self.l = com_height
        self.I = body_pitch_inertia
        self.r = wheel_radius
        self.Jw_each = wheel_inertia_each
        self.Jm = motor_rotor_inertia
        self.G = gear_ratio
        self.bx = longitudinal_viscous or 0.0
        self.Fc = longitudinal_coulomb or 0.0
        self.bp = pitch_viscous or 0.0


    def 