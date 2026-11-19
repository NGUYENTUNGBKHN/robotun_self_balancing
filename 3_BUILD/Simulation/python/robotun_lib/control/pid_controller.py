"""
pid_controller.py

Proportional, integral, derivation

"""

class pid_controller:

    def __init__(self, Kp, Ki, Kd, target, init_state=0.0) -> None:
        """
        Kp, Ki, Kd  : PID gains
        target      : setpoint 
        init_state  : Initialized state
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = target
        self.last_state = init_state


    def set_target(self, target):
        self.target = target

    def get_ctrl(self, value, dt):
        
