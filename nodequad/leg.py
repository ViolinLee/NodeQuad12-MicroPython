from math import sin, cos, pi, atan2, sqrt, acos
from setting import *
from geometry import FastTransform


class Leg:
    """Leg封装腿部数学运算、模型转换"""
    def __init__(self, servo, leg_index):
        self.leg_index = leg_index
        self.servo = servo
        if self.leg_index == 0:  # -45 or 315 degree
            self.mount_position = (leg_mount_x, leg_mount_y, 0)
            self._local_conv = FastTransform.rotate45
            self._world_conv = FastTransform.rotate315
            self._tip_pos_local = (p1_x, p1_y, p1_z)
        elif self.leg_index == 1:  # 45 or -315 degree
            self.mount_position = (-leg_mount_x, leg_mount_y, 0)
            self._local_conv = FastTransform.rotate315
            self._world_conv = FastTransform.rotate45
            self._tip_pos_local = (p2_x, p2_y, p2_z)
        elif self.leg_index == 2:  # 135 or -225 degree
            self.mount_position = (-leg_mount_x, -leg_mount_y, 0)
            self._local_conv = FastTransform.rotate225
            self._world_conv = FastTransform.rotate135
            self._tip_pos_local = (p3_x, p3_y, p3_z)
        elif self.leg_index == 3:  # 225 or -135 degree
            self.mount_position = (leg_mount_x, -leg_mount_y, 0)
            self._local_conv = FastTransform.rotate135
            self._world_conv = FastTransform.rotate225
            self._tip_pos_local = (p4_x, p4_y, p4_z)
        self._tip_pos = self.translate2world(self._tip_pos_local)
        self.initial_tip_pos_local = self._tip_pos_local

    @staticmethod
    def local_ik(local_tip_pose):
        angles = [0.0] * 3

        x = local_tip_pose[0]
        y = local_tip_pose[1]
        angles[0] = atan2(x, y) * 180 / pi

        x = sqrt(x ** 2 + y ** 2) - leg_joint1_2joint2
        y = -local_tip_pose[2]  # Note: Z-axis Down
        ar = atan2(y, x)
        lr2 = x ** 2 + y ** 2
        lr = sqrt(lr2)
        a1 = acos((lr2 + leg_joint2_2joint3 ** 2 - leg_joint3_2tip ** 2) / (2 * leg_joint2_2joint3 * lr))
        a2 = acos((lr2 - leg_joint2_2joint3 ** 2 + leg_joint3_2tip ** 2) / (2 * leg_joint3_2tip * lr))
        angles[1] = (ar + a1) * 180 / pi
        angles[2] = 90 - ((a1 + a2) * 180 / pi)

        return angles

    @staticmethod
    def local_fk(joint_degree):
        """Only for simulation&visualization"""
        radians = [angle * pi / 180 for angle in joint_degree]
        joint1_pos = [0, 0, 0]
        joint2_pos = [joint1_pos[i] + [leg_joint1_2joint2 * sin(radians[0]), leg_joint1_2joint2 * cos(radians[0]), 0][i]
                      for i in range(3)]
        joint3_pos = [joint2_pos[i] + [leg_joint2_2joint3 * cos(radians[1]) * sin(radians[0]),
                                       leg_joint2_2joint3 * cos(radians[1]) * cos(radians[0]),
                                       -leg_joint2_2joint3 * sin(radians[1])][i] for i in range(3)]  # Note: Z-axis Down
        tip_pos = [joint3_pos[i] + [cos(radians[1] + radians[2] - pi / 2) * leg_joint3_2tip * sin(radians[0]),
                                    cos(radians[1] + radians[2] - pi / 2) * leg_joint3_2tip * cos(radians[0]),
                                    -sin(radians[1] + radians[2] - pi / 2) * leg_joint3_2tip][i] for i in range(3)]

        leg_vectors_local = [[0, 0, 0], joint1_pos, joint2_pos, joint3_pos, tip_pos]

        return leg_vectors_local

    def translate2local(self, world_point):
        """coordinate translation: world to local"""
        return self._local_conv([world_point[i] - self.mount_position[i] for i in range(3)])

    def translate2world(self, local_point):
        """coordinate translation: local to world"""
        return [self._world_conv(local_point)[i] + self.mount_position[i] for i in range(3)]

    def __move(self, target_point_local):
        """servo/hardware interface"""
        angles = Leg.local_ik(target_point_local)
        for joint_index in range(3):
            self.servo.set_angle(self.leg_index, joint_index, angles[joint_index])

    def move_tip(self, target_point_world):
        """word coordiante system (default)"""
        if target_point_world == self._tip_pos:
            return
        dest_local = self.translate2local(target_point_world)
        # logging info
        self.__move(dest_local)
        self._tip_pos = target_point_world
        self._tip_pos_local = dest_local

    def move_tip_local(self, target_point_local):
        """api: leg moving"""
        if target_point_local == self._tip_pos_local:
            return
        dest_world = self.translate2world(target_point_local)
        self.__move(target_point_local)
        self._tip_pos = dest_world
        self._tip_pos_local = target_point_local

    def move_joints_directly(self, target_joint_angles):
        """直接驱动腿部关节角，不会改变腿实例的运动学变量/属性，在舵机校正阶段使用"""
        for joint_index in range(3):
            self.servo.set_angle(self.leg_index, joint_index, target_joint_angles[joint_index])


