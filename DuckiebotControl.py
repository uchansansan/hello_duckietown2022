import numpy as np


class DuckiebotControl:
    k_p = 2.0
    k_r = 6.5
    k_d = 2.6
    k_a = 2.6
    v_0 = 0.24

    def __init__(self, frame_skip=1):
        self._DEFAULT_TICS = 90 / frame_skip

        self.distance = 0.0
        self.angle = 0.0
        self._previos_distance = 0.0
        self._previos_angle = 0.0

        self._tics = self._DEFAULT_TICS

    def update(self, angle, distance):
        self.distance = distance
        self.angle = angle
        # print(self._state)
        if self._tics >= 1:
            self._tics -= 1
        else:
            self._tics = self._DEFAULT_TICS

    def get_action(self):
        """
        calculate next action
        """
        action = np.array([0, 0])

        regul = self.angle
        dx = self.distance - self._previos_distance
        da = self.angle - self._previos_angle
        v_l = self.v_0 - self.k_p * self.distance - regul * self.k_r - self.k_d * dx - self.k_a * da
        v_r = self.v_0 + self.k_p * self.distance + regul * self.k_r + self.k_d * dx + self.k_a * da

        action = np.array([(v_r + v_l) / 2, (v_r - v_l) / 2])  # like PD in basic_control
        self._previos_distance = self.distance
        self._previos_angle = self.angle
        return action
