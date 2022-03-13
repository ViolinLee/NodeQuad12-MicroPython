from pca9685 import PCA9685
from math import radians


class Servo:
    """Servo绫诲皝瑁匬CA9685鐣欎笅涓婂眰API"""

    def __init__(self, i2c, address, pulse_min, pulse_max, pulse_freq, degrees=180):
        self.period = 1000000 / pulse_freq  # 周期/T(s) = 1/freq，freq=50Hz-->周期/T = 20000(us) = 20(ms)
        self.min_duty = self._us2duty(pulse_min)
        self.max_duty = self._us2duty(pulse_max)
        self.pca9685 = PCA9685(i2c, address)  # Hardware Interface
        self.pulse_min, self.pulse_max = pulse_min, pulse_max
        self.degrees = degrees
        self.offset = [[0, 0, 0] for i in range(4)]  # not shallow copy: [[0, 0, 0]] * 3
        self.pca9685.freq(pulse_freq)

    def set_offset(self, offset):
        self.offset = offset

    def km_angle2pulse(self, km_angle, reverse):
        # return interp(correct * km_angle, [-90, 90], [pulse_min, pulse_max])  # 注意运动学定义的0°对应舵机90°位置
        return (self.pulse_min + self.pulse_max) / 2 + reverse * km_angle * ((self.pulse_max - self.pulse_min) / 180)

    def _us2duty(self, value):
        # 一周期(20000us)用12位精度来表示。value(us)转换为占用周期数（分数/小数，即不到一个周期，故 value < 20000us = 20ms）
        return int(4095 * value / self.period)

    def position(self, index, degrees=None, radians=None, us=None, duty=None):
        span = self.max_duty - self.min_duty
        if degrees is not None:
            duty = self.min_duty + span * degrees / self.degrees
        elif radians is not None:
            duty = self.min_duty + span * radians / radians(self.degrees)
        elif us is not None:
            duty = self._us2duty(us)
        elif duty is not None:
            pass
        else:
            return self.pca9685.duty(index)
        duty = min(self.max_duty, max(self.min_duty, int(duty)))
        self.pca9685.duty(index, duty)

    def set_angle(self, leg_index, part_index, km_angle):
        # switch left, right pwm
        if leg_index == 0:
            pwm_index = 0 + part_index
        elif leg_index == 1:
            pwm_index = 3 + part_index
        elif leg_index == 2:
            pwm_index = 6 + part_index
        elif leg_index == 3:
            pwm_index = 9 + part_index
        else:
            raise ValueError

        km_angle_corrected = km_angle + self.offset[leg_index][part_index]
        inverse = -1 if part_index == 1 else 1  # compensate for kinematics model
        inverse = inverse * -1 if leg_index in [1, 3] and part_index in [1, 2] else inverse  # compensate for model mirror

        pulse = min(max(int(self.km_angle2pulse(km_angle_corrected, inverse)), self.pulse_min), self.pulse_max)

        self.position(pwm_index, us=pulse)




