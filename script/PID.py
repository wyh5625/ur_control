# coding=utf-8
import numpy as np
import time


class PID:
    def __init__(self, P=0.0, I=0.0, D=0.0, out_max=np.inf):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.clear()
        self.out_max = out_max
        self.out_min = -self.out_max
        self.current_value = np.array(0.0)
        self.target_value = np.array(0.0)

    def clear(self):
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.target_value = np.array(0.0)
        self.last_error = 0.0
        self.int_error = 0.0
        self.windup_guard = 20.0
        self.output = 0.0
        self.output_limited = 0.0

    def update(self, target_value, current_value, delta_time=None):
        self.target_value = target_value
        self.current_value = current_value
        error = self.target_value - current_value
        return self.update_with_err(error, delta_time)

    def update_with_err(self, error, delta_time=None):
        self.current_time = time.time()

        if delta_time is None:
            delta_time = self.current_time - self.last_time

        delta_error = error - self.last_error

        # 比例
        self.PTerm = self.Kp * error

        # 积分
        self.ITerm += error * delta_time

        # 积分限幅
        np.clip(self.ITerm, -self.windup_guard, self.windup_guard)

        # 微分
        self.DTerm = 0.0
        if delta_time > 0:
            self.DTerm = delta_error / delta_time

        self.last_time = self.current_time
        self.last_error = error

        # Output
        self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
        self.output_limited = np.clip(self.output, self.out_min, self.out_max)
        return self.output_limited

    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time