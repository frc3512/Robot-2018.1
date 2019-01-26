#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")

import control as cnt
import frccontrol as frccnt
import matplotlib.pyplot as plt
import numpy as np


def drivetrain(motor, num_motors, m, r, rb, J, Gl, Gr):
    """Returns the state-space model for a drivetrain.

    States: [[left velocity], [right velocity]]
    Inputs: [[left voltage], [right voltage]]
    Outputs: [[left velocity], [right velocity]]

    Keyword arguments:
    motor -- instance of DcBrushedMotor
    um_motors -- number of motors driving the mechanism
    m -- mass of robot in kg
    r -- radius of wheels in meters
    rb -- radius of robot in meters
    J -- moment of inertia of the drivetrain in kg-m^2
    Gl -- gear ratio of left side of drivetrain
    Gr -- gear ratio of right side of drivetrain

    Returns:
    StateSpace instance containing continuous model
    """
    motor = frccnt.models.gearbox(motor, num_motors)

    C1 = -Gl**2 * motor.Kt / (motor.Kv * motor.R * r**2)
    C2 = Gl * motor.Kt / (motor.R * r)
    C3 = -Gr**2 * motor.Kt / (motor.Kv * motor.R * r**2)
    C4 = Gr * motor.Kt / (motor.R * r)
    # fmt: off
    A = np.matrix([[(1 / m + rb**2 / J) * C1, (1 / m - rb**2 / J) * C3],
                   [(1 / m - rb**2 / J) * C1, (1 / m + rb**2 / J) * C3]])
    B = np.matrix([[(1 / m + rb**2 / J) * C2, (1 / m - rb**2 / J) * C4],
                   [(1 / m - rb**2 / J) * C2, (1 / m + rb**2 / J) * C4]])
    C = np.matrix([[1, 0], [0, 1]])
    D = np.matrix([[0, 0], [0, 0]])
    # fmt: on

    return cnt.ss(A, B, C, D)


class Drivetrain(frccnt.System):

    def __init__(self, dt):
        """Drivetrain subsystem.


        Keyword arguments:
        dt -- time between model/controller updates
        """
        state_labels = [("Left velocity", "m/s"), ("Right velocity", "m/s")]
        u_labels = [("Left voltage", "V"), ("Right voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        self.in_low_gear = False

        # Number of motors per side
        self.num_motors = 2.0

        # High and low gear ratios of drivetrain
        Ghigh = 72.0 / 12.0

        # Drivetrain mass in kg
        self.m = 64
        # Radius of wheels in meters
        self.r = 0.0746125
        # Radius of robot in meters
        self.rb = 0.6096 / 2.0
        # Moment of inertia of the drivetrain in kg-m^2
        self.J = 4.0

        # Gear ratios of left and right sides of drivetrain respectively
        if self.in_low_gear:
            self.Gl = Glow
            self.Gr = Glow
        else:
            self.Gl = Ghigh
            self.Gr = Ghigh

        self.model = drivetrain(
            frccnt.models.MOTOR_CIM,
            self.num_motors,
            self.m,
            self.r,
            self.rb,
            self.J,
            self.Gl,
            self.Gr,
        )
        u_min = np.matrix([[-12.0], [-12.0]])
        u_max = np.matrix([[12.0], [12.0]])
        frccnt.System.__init__(self, self.model, u_min, u_max, dt)

        if self.in_low_gear:
            q_vel = 1.0
        else:
            q_vel = 0.95

        q = [q_vel, q_vel]
        r = [12.0, 12.0]
        self.design_lqr(q, r)

        qff_vel = 0.01
        self.design_two_state_feedforward([qff_vel, qff_vel], [12, 12])

        q_vel = 1.0
        r_vel = 0.01
        self.design_kalman_filter([q_vel, q_vel], [r_vel, r_vel])

        print("ctrb cond =", np.linalg.cond(cnt.ctrb(self.sysd.A, self.sysd.B)))


def main():
    dt = 0.00505
    drivetrain = Drivetrain(dt)
    drivetrain.export_cpp_coeffs("Drivetrain", "control/")

    if "--save-plots" in sys.argv or "--noninteractive" not in sys.argv:
        try:
            import slycot

            plt.figure(1)
            drivetrain.plot_pzmaps()
        except ImportError:  # Slycot unavailable. Can't show pzmaps.
            pass
    if "--save-plots" in sys.argv:
        plt.savefig("drivetrain_pzmaps.svg")

    t, xprof, vprof, aprof = frccnt.generate_s_curve_profile(
        max_v=4.0, max_a=3.5, time_to_max_a=1.0, dt=dt, goal=50.0)

    # Generate references for simulation
    refs = []
    for i in range(len(t)):
        r = np.matrix([[xprof[i]], [vprof[i]], [xprof[i]], [vprof[i]]])
        refs.append(r)

    if "--save-plots" in sys.argv or "--noninteractive" not in sys.argv:
        plt.figure(2)
        state_rec, ref_rec, u_rec = drivetrain.generate_time_responses(t, refs)
        drivetrain.plot_time_responses(t, state_rec, ref_rec, u_rec)
    if "--save-plots" in sys.argv:
        plt.savefig("drivetrain_response.svg")
    if "--noninteractive" not in sys.argv:
        plt.show()


if __name__ == "__main__":
    main()
