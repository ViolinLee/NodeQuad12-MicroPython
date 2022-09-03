import json
from setting import panel_html_str, calib_html_str
from socket import socket, AF_INET, SOCK_STREAM, SOL_SOCKET, SO_REUSEADDR
from gait import MODE_MOVE, MODE_POSE
from gait import GAIT_TROT, GAIT_WALK, GAIT_GALLOP, GAIT_CREEP
from gait import MOVE_STANDBY, MOVE_FORWARD, MOVE_BACKWARD, MOVE_LEFTSHIFT, MOVE_RIGHTSHIFT, MOVE_LEFTROTATE, \
    MOVE_RIGHTROTATE
from utime import sleep_ms


class WebController:
    """遥控前端逻辑和生成遥控量"""

    def __init__(self, sta_ip):
        self.sta_ip = sta_ip
        self.calibration_data = [[0.0, 0.0, 0.0] for i in range(4)]  # direct angle (rad)
        self.rc_calibration = 0
        self.rc_mode = MODE_MOVE
        self.rc_gait_mode = GAIT_TROT
        self.rc_moving_status = MOVE_STANDBY
        self.rc_pose = [0, 0, 0, 0, 0, 0]  # x, y, z, roll, pitch, yaw

    @staticmethod
    def pose_cal(component, joystick_range, rotate_range):
        pose = [int(component[i] * rotate_range/joystick_range) for i in range(len(component))]
        return pose

    def load_calibration(self, json_path):
        with open(json_path, 'r') as f:
            json_data = json.load(f)
            self.calibration_data = json_data['calibration']
        print("Load calibration data: {0}\n}".format(self.calibration_data))

    def save_calibration(self, json_path):
        with open(json_path, 'w') as f:
            json_dict = {'calibration': self.calibration_data}
            json.dump(json_dict, f)
        print("Save calibration data: {0}\n".format(self.calibration_data))

    def reset_rc_except_mode(self):
        self.rc_gait_mode = GAIT_TROT
        self.rc_moving_status = MOVE_STANDBY
        self.rc_pose = [0, 0, 0, 0, 0, 0]

    def process_panel(self, panel_req):
        try:
            header, _, json_string = panel_req.partition('\r\n\r\n')
            ctrl_quantity = json.loads(json_string)  # {'button': 'POSE', 'joy.x': 100, 'joy.x': 150, 'joy.z': 50}
        except:
            # print(panel_req)
            return
        
        button = ctrl_quantity['button'] if 'button' in ctrl_quantity.keys() else None
        joy_x = ctrl_quantity['joy.x'] if 'joy.x' in ctrl_quantity.keys() else self.rc_pose[3]
        joy_y = ctrl_quantity['joy.y'] if 'joy.y' in ctrl_quantity.keys() else self.rc_pose[4]
        joy_z = ctrl_quantity['joy.z'] if 'joy.z' in ctrl_quantity.keys() else self.rc_pose[5]
        calibration = ctrl_quantity['calibration'] if 'calibration' in ctrl_quantity.keys() else None

        if button == 'CALIBRATESTART':
            print(button)
            self.rc_calibration = 1
        elif button == 'CALIBRATESAVE':
            print(button)
            self.rc_calibration = 0
        else:
            pass

        if self.rc_calibration == 1:
            if calibration is not None:
                calibration_int = [int(str_num) for str_num in calibration]
                temp = [calibration_int[3 * i: 3 * i + 3] for i in range(4) if i % 2 == 1]
                temp_r = [calibration_int[3 * i: 3 * i + 3] for i in range(4) if i % 2 == 0]
                temp_r.reverse()
                self.calibration_data = temp + temp_r
            else:
                pass
        else:
            if button is not None and button in ['POSE', 'MOVE']:
                self.rc_mode = self.get_mode(button)
                self.reset_rc_except_mode()

            if self.rc_mode == MODE_MOVE:
                if button in ['STANDBY', 'FORWARD', 'BACKWARD', 'LEFTSHIFT', 'RIGHTSHIFT', 'LEFTTURN', 'RIGHTTURN']:
                    self.rc_moving_status = self.get_moving_status(button)
                elif button in ['TROT', 'WALK', 'GALLOP', 'CREEP']:
                    self.rc_gait_mode = self.get_gait_mode(button)
                else:
                    pass
            elif self.rc_mode == MODE_POSE:
                if button == 'STANDBY':
                    self.reset_rc_except_mode()
                elif button in ['FORWARD', 'BACKWARD', 'LEFTSHIFT', 'RIGHTSHIFT', 'LEFTTURN', 'RIGHTTURN']:
                    shift_inc = self.get_shift_increment(button)
                    for i, shift in enumerate(shift_inc[:3]):
                        self.rc_pose[i] = min(max(self.rc_pose[i] + shift, -30), 30)
                self.rc_pose[3:] = [joy_x, joy_y, joy_z]
            else:
                pass

    @classmethod
    def get_mode(cls, button):
        transitions_mode = {'POSE': MODE_POSE, 'MOVE': MODE_MOVE}
        return transitions_mode[button]

    @classmethod
    def get_gait_mode(cls, button):
        transitions_gait_mode = {'TROT': GAIT_TROT,
                                 'WALK': GAIT_WALK,
                                 'GALLOP': GAIT_GALLOP,
                                 'CREEP': GAIT_CREEP}
        return transitions_gait_mode[button]

    @classmethod
    def get_moving_status(cls, button):
        transitions_moving_status = {'STANDBY': MOVE_STANDBY,
                                     'FORWARD': MOVE_FORWARD,
                                     'BACKWARD': MOVE_BACKWARD,
                                     'LEFTSHIFT': MOVE_LEFTSHIFT,
                                     'RIGHTSHIFT': MOVE_RIGHTSHIFT,
                                     'LEFTTURN': MOVE_LEFTROTATE,
                                     'RIGHTTURN': MOVE_RIGHTROTATE}
        return transitions_moving_status[button]
    
    @classmethod
    def get_shift_increment(cls, button):
        # button in ['FORWARD', 'BACKWARD', 'LEFTSHIFT', 'RIGHTSHIFT', 'LEFTTURN', 'RIGHTTURN']
        if button == 'FORWARD':
            inc = [10, 0, 0]
        elif button == 'BACKWARD':
            inc = [-10, 0, 0]
        elif button == 'LEFTSHIFT':
            inc = [0, -10, 0]
        elif button == 'RIGHTSHIFT':
            inc = [0, 10, 0]
        elif button == 'LEFTTURN':
            inc = [0, 0, -10]
        elif button == 'RIGHTTURN':
            inc = [0, 0, 10]
        else:
            raise ValueError
        return inc

    def loop(self):
        # Setup Socket WebServer
        s = socket(AF_INET, SOCK_STREAM)
        s.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        s.bind((self.sta_ip, 80))
        s.listen(1)

        # Main Loop
        while True:
            try:
                # Accept request from clients
                conn, addr = s.accept()
            except OSError:
                    continue
            req = conn.recv(1024).decode()  # req = str(conn.recv(1024))
            
            # Parse Request and Process Remote Control Panel Input from Client
            self.process_panel(req)

            # conn.sendall('HTTP/1.1 200 OK\nConnection: close\nServer: FireBeetle\nContent-Type: text/html\n\n'.encode())
            if req.find('favicon.ico') > -1:  # Filter
                conn.close()
                continue
            elif req.find('calibration_page') > -1:
                try:
                    conn.sendall(calib_html_str)
                    conn.close()
                except OSError:
                    continue
            else:
                try:
                    # Response and Close Socket
                    conn.sendall(panel_html_str)
                    conn.close()
                except OSError:
                    continue

            # print("Mode: ", self.rc_mode, "Gait:", self.rc_gait_mode, "Status: ", self.rc_moving_status, "Pose:", self.rc_pose)














