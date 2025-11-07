#!/usr/bin/env python3

from typing import Tuple
import numpy as np

class PIDController():
    def __init__(self):

        # We will initialize some variables that might be useful
        self.prev_e_heading = 0.0
        self.prev_e_offset = 0.0
        self.prev_int_heading = 0.0
        self.prev_int_offset = 0.0

        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0


    def HeadingControl(self,
                       v_ref: float,
                       theta_ref: float,
                       theta_curr: float,
                       delta_t: float
    ) -> Tuple[float, float]:
        """
        PID performing heading control.
        Args:
            v_ref:      reference velocity.
            theta_ref:  reference heading pose.
            theta_curr: the current estimated heading.
            delta_t:    time interval since last call.
        Returns:
            v:          linear velocity of the Duckiebot
            omega:      angular velocity of the Duckiebot
        """

        # TODO: implement a PID controller to track the reference heading
        # feel free to make use of the global variables:
        # self.kp, self.ki, and self. kd, which are
        # set either by the notebook or from noVNC
        # as well as self_prev_int_heading to track the integral term
        # self.prev_e_heading the previous error. But note that you
        # should be the one to update them also.
        # initiate the error

        # --- Compute heading error
        e_heading = theta_ref - theta_curr

        # --- Wrap angle to [-pi, pi]
        e_heading = (e_heading + np.pi) % (2 * np.pi) - np.pi

        # --- Update integral term (unclamped)
        self.prev_int_heading += np.clip(e_heading * delta_t, -0.5, 0.5)

        # --- Derivative term
        d_heading = (e_heading - self.prev_e_heading) / delta_t if delta_t > 0 else 0.0

        # --- Compute individual PID components
        p_term = self.kp * e_heading
        i_term = self.ki * self.prev_int_heading
        d_term = self.kd * d_heading

        # --- Combine control output
        omega = p_term + i_term + d_term

        # Debugging
        # print('p:', p_term)
        # print('i', i_term)
        # print('d:', d_term)

        # --- Update previous error
        self.prev_e_heading = e_heading

        # --- Constant linear velocity
        v = v_ref

        return v, omega

    def OffsetControl(self,
                      v_ref: float,
                      y_ref: float,
                      y_curr: float,
                      delta_t: float
                      ) -> Tuple[float, float]:
        """
        PID performing lateral offset control.
        Args:
            v_ref:      linear Duckiebot speed.
            y_ref:      reference heading pose.
            y_curr:     the current estimated "y" coordinate (offset)
            delta_t:    time interval since last call.
        Returns:
            v:          linear velocity of the Duckiebot
            omega:      angular velocity of the Duckiebot
        """

        # TODO: implement a PID controller to track the reference lateral offset
        # feel free to make use of the global variables:
        # self.kp, self.ki, and self. kd, which are
        # set either by the notebook or from noVNC
        # as well as self_prev_int_offset to track the integral term
        # self.prev_e_offset the previous error. But note that you
        # should be the one to update them also.

        # --- Compute lateral offset error
        e_offset = y_ref - y_curr

        # self.prev_int_offset += np.clip(e_offset * delta_t, -0.5, 0.5)
        # --- Update integral term (unclamped)
        self.prev_int_offset += e_offset * delta_t
        # self.prev_int_offset = np.clip(self.prev_int_offset, -1.0, 1.0)

        # --- Derivative term
        d_offset = (e_offset - self.prev_e_offset) / delta_t if delta_t > 0 else 0.0

        # --- Compute PID components
        p_term = self.kp * e_offset
        i_term = self.ki * self.prev_int_offset
        d_term = self.kd * d_offset

        # print('p:', p_term)
        # print('i:', i_term)
        # print('d:', d_term)

        # --- Clamp *only* the integral contribution
        # i_term = np.clip(i_term, -0.005, 0.005)  # Adjust this range based on limits

        # --- Combine control output
        omega = p_term + i_term + d_term

        # Debugging
        # print('p:', p_term)
        # print('i', i_term)
        # print('d:', d_term)

        # --- Update previous error
        self.prev_e_offset = e_offset

        # --- Constant linear velocity
        v = v_ref

        return v, omega

    def SetGains(self, kp: float, ki: float, kd: float) -> None:
        # Set the PID gains
        self.prev_int_offset = 0
        self.prev_int_heading = 0
        self.kp = kp
        self.ki = ki
        self.kd = kd