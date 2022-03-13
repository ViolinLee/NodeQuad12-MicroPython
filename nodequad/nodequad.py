import json
from leg import Leg
from gait import Gait
from Servo import Servo
from utime import sleep_ms
from geometry import *
from gait import MODE_MOVE, MODE_POSE
from gait import GAIT_TROT
from gait import MOVE_STANDBY
from controller import WebController
from setting import calibration_path
from math import pi


class NodeQuad:
    """
    四足机器人类 NodeQuad--Leg(kinematics + transform)--Servo(geometry)--PCA9685(iic)
                        --Gait(trajectory)
    """
    def __init__(self, sta_ip, i2c, address, pulse_min, pulse_max, pulse_freq):
        # is calibrating
        self.calibration = 0  # 0-normal 1-calibration

        # action mode
        self.mode = MODE_MOVE
        # status
        self.moving_status = MOVE_STANDBY
        # hardware
        self.servo = Servo(i2c, address, pulse_min, pulse_max, pulse_freq)
        # kinematics
        self.legs = [Leg(self.servo, i) for i in range(4)]
        # gait
        self.gait = Gait()
        self.gait_mode = GAIT_TROT  # 0-Trot 1-Walk 2-Pace 3-Creep
        self.gait_path = self.gait.gen_path(self.gait_mode, self.moving_status)
        self.path_step_beginid = 0
        self.path_step_id = 0
        # web controller
        self.web_controller = WebController(sta_ip)

        # other setting
        self.faster = False  # 通过减少采样点来提高移动速度（预留）
    
    def init(self):
        self.load_calibration(calibration_path)
        print("NodeQuad init done.")
        
    def load_calibration(self, json_path):
        with open(json_path, 'r') as f:
            json_data = json.load(f)
            calibration_data = json_data['calibration']
            self.servo.set_offset(calibration_data)
        print("Load calibration data: {0}\n".format(calibration_data))
        
    def save_calibration(self, json_path):
        with open(json_path, 'w') as f:
            json_dict = {'calibration': self.servo.offset}
            json.dump(json_dict, f)
        print("Save calibration data: {0}\n".format(self.servo.offset))

    def update_iscalibration(self):
        if self.calibration != self.web_controller.rc_calibration:
            self.calibration = self.web_controller.rc_calibration

    def update_gait_mode(self):
        if self.web_controller.rc_gait_mode != self.gait_mode:
            self.gait_mode = self.web_controller.rc_gait_mode
            self.path_step_id = 0  # in potential!
        else:
            pass  # 避免每次都重新计算轨迹以节约计算资源

    def update_moving_status(self):
        if self.web_controller.rc_moving_status != self.moving_status:  #  and self.path_step_id == self.path_step_beginid:
            self.moving_status = self.web_controller.rc_moving_status
            self.gait_path = self.gait.gen_path(self.gait_mode, self.moving_status)
            self.path_step_id = 0  # in potential!

    def update_mode(self):
        if self.mode != self.web_controller.rc_mode:
            self.mode = self.web_controller.rc_mode
            self.path_step_id = 0
            
    def map_rc_pose(self):
        trans = self.web_controller.rc_pose[:3]
        orns = [orn * 15 / 100 * pi/180 for orn in self.web_controller.rc_pose[3:]]
        return trans, orns

    def pose_transform(self, pos_tuple, orn_tuple):
        """
        :param pos_tuple: (x, y, z)  orn_tuple: (roll, pitch, yaw)
        :return:
        """
        # 1 solve RT
        matrix_body_tran = pose2tran(pos_tuple, orn_tuple)

        # 2 body-frame to coxa-frame vector (on world frame)
        fr_body2coxa = pos2vec(self.legs[0].mount_position)
        br_body2coxa = pos2vec(self.legs[1].mount_position)
        bl_body2coxa = pos2vec(self.legs[2].mount_position)
        fl_body2coxa = pos2vec(self.legs[3].mount_position)

        # 3 body-frame to tip-frame vector (on world-frame)
        fr_body2tip = pos2vec(self.legs[0].initial_tip_pos_local)
        br_body2tip = pos2vec(self.legs[1].initial_tip_pos_local)
        bl_body2tip = pos2vec(self.legs[2].initial_tip_pos_local)
        fl_body2tip = pos2vec(self.legs[3].initial_tip_pos_local)

        # 4 body-frame to coxa-frame vector rotated with the body (on world-frame)
        fr_body2coxa_rotated = matrix_multiply(matrix_body_tran, fr_body2coxa, 4, 4, 1)
        br_body2coxa_rotated = matrix_multiply(matrix_body_tran, br_body2coxa, 4, 4, 1)
        bl_body2coxa_rotated = matrix_multiply(matrix_body_tran, bl_body2coxa, 4, 4, 1)
        fl_body2coxa_rotated = matrix_multiply(matrix_body_tran, fl_body2coxa, 4, 4, 1)

        # 5 coxa-frame to tip-frame vector (on world-frame)
        fr_coxa2tip = matrix_subtract(fr_body2tip, fr_body2coxa_rotated, 4, 1)
        br_coxa2tip = matrix_subtract(br_body2tip, br_body2coxa_rotated, 4, 1)
        bl_coxa2tip = matrix_subtract(bl_body2tip, bl_body2coxa_rotated, 4, 1)
        fl_coxa2tip = matrix_subtract(fl_body2tip, fl_body2coxa_rotated, 4, 1)

        # 6 convert to a representation on coxa-frame necessary for IK (coxa-frame)
        matrix_undo_orn_tran = inv_tran(pose2tran((0, 0, 0), orn_tuple))
        fr_coxa2tip_coxa = matrix_multiply(matrix_undo_orn_tran, fr_coxa2tip, 4, 4, 1)
        br_coxa2tip_coxa = matrix_multiply(matrix_undo_orn_tran, br_coxa2tip, 4, 4, 1)
        bl_coxa2tip_coxa = matrix_multiply(matrix_undo_orn_tran, bl_coxa2tip, 4, 4, 1)
        fl_coxa2tip_coxa = matrix_multiply(matrix_undo_orn_tran, fl_coxa2tip, 4, 4, 1)
        
        fr_local2coxa_tran = pose2tran((0, 0, 0), (0, 0, 45*pi/180))
        br_local2coxa_tran = pose2tran((0, 0, 0), (0, 0, 315*pi/180))
        bl_local2coxa_tran = pose2tran((0, 0, 0), (0, 0, 225*pi/180))
        fl_local2coxa_tran = pose2tran((0, 0, 0), (0, 0, 135*pi/180))
        
        fr_tip_local = matrix_multiply(fr_local2coxa_tran, fr_coxa2tip_coxa, 4, 4, 1)
        br_tip_local = matrix_multiply(br_local2coxa_tran, br_coxa2tip_coxa, 4, 4, 1)
        bl_tip_local = matrix_multiply(bl_local2coxa_tran, bl_coxa2tip_coxa, 4, 4, 1)
        fl_tip_local = matrix_multiply(fl_local2coxa_tran, fl_coxa2tip_coxa, 4, 4, 1)

        target_pos_local = [fr_tip_local[:-1],
                            br_tip_local[:-1],
                            bl_tip_local[:-1],
                            fl_tip_local[:-1]]
        return target_pos_local

    def move_legs_tips(self, target_pos, local=True):
        # print(target_pos)
        for leg_index in range(4):
            if local:
                self.legs[leg_index].move_tip_local(target_pos[leg_index])
            else:
                self.legs[leg_index].move_tip(target_pos[leg_index])

    def move_legs_joints(self, target_joint_angles):
        for leg_index in range(4):
            self.legs[leg_index].move_joints_directly(target_joint_angles[leg_index])

    def main_loop(self):
        """主线程：所有对遥控量的再处理都放在这里"""
        while True:
            # for debug
            # print("is calibrating: ", self.calibration)
            # print("mode: ", self.mode, "gait mode: ", self.gait_mode, "moving status: ",self.moving_status , "path id: ", self.path_step_id)
            self.update_iscalibration()

            if self.calibration == 0:  # normal mode
                self.update_mode()
                if self.mode == MODE_MOVE:  # MOVE
                    self.update_gait_mode()
                    self.update_moving_status()

                    target_pos_world = self.gait_path[self.path_step_id]
                    #print(target_pos_world)
                    self.path_step_id = self.path_step_id + 1 if self.path_step_id < len(self.gait_path) - 1 else 0
                    self.move_legs_tips(target_pos_world, local=False)
                    sleep_ms(15)  # 舵机响应时间
                elif self.mode == MODE_POSE:  # POSE
                    # Body姿态 -> 目标足尖坐标
                    x_y_z, roll_pitch_yaw = self.map_rc_pose()   # Joystick Mapping and process
                    # print(x_y_z, roll_pitch_yaw)
                    target_pos_local = self.pose_transform(x_y_z, roll_pitch_yaw)  # Transform
                    # print(target_pos_local)
                    self.move_legs_tips(target_pos_local, local=True)  # IK -> joint
                else:
                    raise ValueError("Unexpected Mode")

            elif self.calibration == 1:  # calibration mode
                while self.calibration:
                    self.update_iscalibration()
                    # print("in nodequad main loop calibrating: ", self.calibration)
                    self.servo.set_offset(self.web_controller.calibration_data)
                    self.move_legs_joints(self.web_controller.calibration_data)  # direct joint
                    sleep_ms(50)
                self.save_calibration(calibration_path)
                self.calibration = 0  # 从calibration模式切换至normal模式不是也不应采取RC遥控的方式：校正结束自动切换
            else:
                pass

            













