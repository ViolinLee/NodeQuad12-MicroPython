from math import sin, cos, pi
from setting import p1_x, p1_y, p1_z, p2_x, p2_y, p2_z, p3_x, p3_y, p3_z, p4_x, p4_y, p4_z
from geometry import FastTransform


GAIT_TROT = 0
GAIT_WALK = 1
GAIT_GALLOP = 2
GAIT_CREEP = 3

MODE_STAND = 00
MODE_MOVE = 10
MODE_POSE = 20

MOVE_STANDBY = 11
MOVE_FORWARD = 12
MOVE_BACKWARD = 13
MOVE_LEFTSHIFT = 14
MOVE_RIGHTSHIFT = 15
MOVE_LEFTROTATE = 16
MOVE_RIGHTROTATE = 17

home_x = [p1_x, p2_x, p3_x, p4_x]
home_y = [p1_y, p2_y, p3_y, p4_y]
home_z = [p1_z, p2_z, p3_z, p4_z]


def left_rotate_path(path: list, rotation_num):
    # method 1
    rotated_path = path[rotation_num:] + path[:rotation_num]

    # method 2
    # rotated_path = [path[(i + rotation_len) % len(path)] for i, x in enumerate(path)]

    # method 3
    # from collections import deque
    # path = deque(path)
    # path.rotate(-rotation_len)
    # rotated_path = list(path)
    return rotated_path


def right_rotate_path(path: list, rotation_num):
    # method 1
    rotated_path = path[(len(path) - rotation_num):] + path[:(len([path]) - rotation_num - 1)]
    return rotated_path


class Gait:
    """步态生成"""
    def __init__(self):
        # gait constants
        self.amplitudeX, self.amplitudeY, self.amplitudeZ = 25, 15, 35
        # movement configuration
        self.frame_time_ms = 20

    def gen_path(self, gait_mode, move_status, gait_speed=0):
        """Generate gait trajectory under world frame."""
        if gait_mode == GAIT_TROT:
            path = self.trot_gait(move_status, gait_speed)
        elif gait_mode == GAIT_WALK:
            path = self.walk_gait(move_status, gait_speed)
        elif gait_mode == GAIT_GALLOP:
            path = self.gallop_gait(move_status, gait_speed)
        elif gait_mode == GAIT_CREEP:
            path = self.creep_gait(move_status, gait_speed)
        else:
            raise ValueError()
        return path

    def formated_path_status(self, fr_path_quad, br_path_quad, bl_path_quad, fl_path_quad, move_status):
        path_quad = [[fr_path_quad[path_id], br_path_quad[path_id], bl_path_quad[path_id], fl_path_quad[path_id]]
                     for path_id in range(len(fr_path_quad))]
        if move_status == MOVE_STANDBY:
            corrected_path = [[[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]]
        elif move_status == MOVE_FORWARD:
            corrected_path = path_quad
        elif move_status == MOVE_BACKWARD:
            corrected_path = [[[-point[0], point[1], point[2]] for point in leg_points] for leg_points in path_quad]
        elif move_status == MOVE_LEFTSHIFT:
            corrected_path = [[FastTransform.rotate270(point) for point in leg_points] for leg_points in path_quad]
        elif move_status == MOVE_RIGHTSHIFT:
            corrected_path = [[FastTransform.rotate90(point) for point in leg_points] for leg_points in path_quad]
        elif move_status == MOVE_LEFTROTATE:
            corrected_path = [[FastTransform.rotate315(leg_points[0]),
                               FastTransform.rotate45(leg_points[1]),
                               FastTransform.rotate135(leg_points[2]),
                               FastTransform.rotate225(leg_points[3])] for leg_points in path_quad]
        elif move_status == MOVE_RIGHTROTATE:
            corrected_path = [[FastTransform.rotate135(leg_points[0]),
                               FastTransform.rotate225(leg_points[1]),
                               FastTransform.rotate315(leg_points[2]),
                               FastTransform.rotate45(leg_points[3])] for leg_points in path_quad]
        else:
            raise ValueError

        path_quad_world = [[[points_set[0][0] + home_x[0], points_set[0][1] + home_y[0], points_set[0][2] + home_z[0]],
                            [points_set[1][0] + home_x[1], points_set[1][1] + home_y[1], points_set[1][2] + home_z[1]],
                            [points_set[2][0] + home_x[2], points_set[2][1] + home_y[2], points_set[2][2] + home_z[2]],
                            [points_set[3][0] + home_x[3], points_set[3][1] + home_y[3], points_set[3][2] + home_z[3]]]
                           for points_set in corrected_path]
        return path_quad_world

    def trot_gait(self, move_status, gait_speed=0):
        """global"""
        duration = 200 if gait_speed == 0 else 400
        # total ticks divided into 2 stages, 'num_ticks' = ticks per stages
        num_ticks = int(duration / self.frame_time_ms / 2)
        # fr_path_quad = [[0.0, 0.0, 0.0]] * num_ticks * 2  # shallow copy
        fr_path_quad = [[0.0, 0.0, 0.0] for i in range(num_ticks*2)]

        for stage_id in range(2):  # A cycle is divided into 2 stages
            for tick_cnt in range(num_ticks):
                interp_id = stage_id * num_ticks + tick_cnt
                if stage_id == 0:
                    fr_path_quad[interp_id][0] = -self.amplitudeX * cos(pi * tick_cnt / num_ticks)
                    fr_path_quad[interp_id][1] = 0.0
                    fr_path_quad[interp_id][2] = abs(self.amplitudeZ) * sin(pi * tick_cnt / num_ticks) * -1
                elif stage_id == 1:
                    fr_path_quad[interp_id][0] = self.amplitudeX * cos(pi * tick_cnt / num_ticks)
                    fr_path_quad[interp_id][1] = 0.0
                    fr_path_quad[interp_id][2] = 0.0
                else:
                    pass

        # Deep copy
        br_path_quad = [[point[0], point[1], point[2]] for point in right_rotate_path(fr_path_quad, num_ticks)]
        bl_path_quad = [[point[0], point[1], point[2]] for point in fr_path_quad]
        fl_path_quad = [[point[0], point[1], point[2]] for point in br_path_quad]

        path_quad_world = self.formated_path_status(fr_path_quad, br_path_quad, bl_path_quad, fl_path_quad, move_status)

        return path_quad_world

    def walk_gait(self, move_status, gait_speed=0):
        duration = 4 * 4 * self.frame_time_ms if gait_speed == 0 else 1280
        # total ticks divided into 4 stages, 'num_ticks' = ticks per stages
        num_ticks = int(duration / self.frame_time_ms / 4)
        fl_path_quad = [[0.0, 0.0, 0.0] for i in range(num_ticks*4)]

        for stage_id in range(4):  # A cycle is divided into 4 stages
            for tick_cnt in range(num_ticks):
                interp_id = stage_id * num_ticks + tick_cnt
                if stage_id == 0:
                    fl_path_quad[interp_id][0] = -self.amplitudeX * cos(pi * tick_cnt / num_ticks) * 1.5
                    fl_path_quad[interp_id][1] = 0.0
                    fl_path_quad[interp_id][2] = abs(self.amplitudeZ) * sin(pi * tick_cnt / num_ticks) * -1
                elif stage_id == 1:
                    fl_path_quad[interp_id][0] = (self.amplitudeX - self.amplitudeX * 2 * (tick_cnt / num_ticks) / 3)  * 1.5
                    fl_path_quad[interp_id][1] = 0.0
                    fl_path_quad[interp_id][2] = 0.0
                elif stage_id == 2:
                    fl_path_quad[interp_id][0] = (self.amplitudeX - self.amplitudeX * 2 * ((num_ticks + tick_cnt) / num_ticks) / 3) * 1.5
                    fl_path_quad[interp_id][1] = 0.0
                    fl_path_quad[interp_id][2] = 0.0
                elif stage_id == 3:
                    fl_path_quad[interp_id][0] = (self.amplitudeX - self.amplitudeX * 2 * ((2 * num_ticks + tick_cnt) / num_ticks) / 3) * 1.5
                    fl_path_quad[interp_id][1] = 0.0
                    fl_path_quad[interp_id][2] = 0.0
                else:
                    pass

        fr_path_quad = [[point[0], point[1], point[2]] for point in right_rotate_path(fl_path_quad, num_ticks * 2)]
        br_path_quad = [[point[0], point[1], point[2]] for point in right_rotate_path(fl_path_quad, num_ticks * 1)]
        bl_path_quad = [[point[0], point[1], point[2]] for point in right_rotate_path(fl_path_quad, num_ticks * 3)]

        path_quad_world = self.formated_path_status(fr_path_quad, br_path_quad, bl_path_quad, fl_path_quad, move_status)

        return path_quad_world

    def gallop_gait(self, move_status, gait_speed=0):
        duration = 4 * 4 * self.frame_time_ms if gait_speed == 0 else 1280
        # total ticks divided into 4 stages, 'num_ticks' = ticks per stages
        num_ticks = int(duration / self.frame_time_ms / 4)
        fl_path_quad = [[0.0, 0.0, 0.0] for i in range(num_ticks*4)]

        for stage_id in range(4):  # A cycle is divided into 4 stages
            for tick_cnt in range(num_ticks):
                interp_id = stage_id * num_ticks + tick_cnt
                if stage_id == 0:
                    fl_path_quad[interp_id][0] = -self.amplitudeX * cos(pi * tick_cnt / num_ticks) * 1.5
                    fl_path_quad[interp_id][1] = 0.0
                    fl_path_quad[interp_id][2] = abs(self.amplitudeZ) * sin(pi * tick_cnt / num_ticks) * -1
                elif stage_id == 1:
                    fl_path_quad[interp_id][0] = (self.amplitudeX - self.amplitudeX * 2 * (tick_cnt / num_ticks) / 3) * 1.5
                    fl_path_quad[interp_id][1] = 0.0
                    fl_path_quad[interp_id][2] = 0.0
                elif stage_id == 2:
                    fl_path_quad[interp_id][0] = (self.amplitudeX - self.amplitudeX * 2 * ((num_ticks + tick_cnt) / num_ticks) / 3) * 1.5
                    fl_path_quad[interp_id][1] = 0.0
                    fl_path_quad[interp_id][2] = 0.0
                elif stage_id == 3:
                    fl_path_quad[interp_id][0] = (self.amplitudeX - self.amplitudeX * 2 * ((2 * num_ticks + tick_cnt) / num_ticks) / 3) * 1.5
                    fl_path_quad[interp_id][1] = 0.0
                    fl_path_quad[interp_id][2] = 0.0
                else:
                    pass

        # Rotary Gallop (Not Transverse Gallop)
        fr_path_quad = [[point[0], point[1], point[2]] for point in right_rotate_path(fl_path_quad, num_ticks * 1)]
        br_path_quad = [[point[0], point[1], point[2]] for point in right_rotate_path(fl_path_quad, num_ticks * 2)]
        bl_path_quad = [[point[0], point[1], point[2]] for point in right_rotate_path(fl_path_quad, num_ticks * 3)]

        path_quad_world = self.formated_path_status(fr_path_quad, br_path_quad, bl_path_quad, fl_path_quad, move_status)

        return path_quad_world

    def creep_gait(self, move_status, gait_speed=0):
        """global"""
        duration = 720 if gait_speed == 0 else 1440
        # total ticks divided into 6 stages, 'num_ticks' = ticks per stages
        num_ticks = int(duration / self.frame_time_ms / 6)
        fr_path_quad, bl_path_quad = [[0.0, 0.0, 0.0] for i in range(num_ticks*6)], [[0.0, 0.0, 0.0] for i in range(num_ticks*6)]

        for stage_id in range(6):  # A cycle is divided into 2 stages
            for tick_cnt in range(num_ticks):
                interp_id = stage_id * num_ticks + tick_cnt
                if stage_id == 0:
                    # home-amplitude到home+amplitude
                    fr_path_quad[interp_id][0] = -self.amplitudeX * cos(pi * tick_cnt / num_ticks) * 2.0
                    fr_path_quad[interp_id][1] = 0.0
                    fr_path_quad[interp_id][2] = abs(self.amplitudeZ) * sin(pi * tick_cnt / num_ticks) * -1.5
                    # 保持home
                    bl_path_quad[interp_id][0] = 0.0
                    bl_path_quad[interp_id][1] = 0.0
                    bl_path_quad[interp_id][2] = 0.0
                elif stage_id == 1:
                    # home+amplitude到home
                    fr_path_quad[interp_id][0] = self.amplitudeX * cos(pi / 2 * tick_cnt / num_ticks) * 2.0
                    fr_path_quad[interp_id][1] = 0.0
                    fr_path_quad[interp_id][2] = 0.0
                    # home往后到home-amplitude
                    bl_path_quad[interp_id][0] = -self.amplitudeX * sin(pi / 2 * tick_cnt / num_ticks) * 2.0
                    bl_path_quad[interp_id][1] = 0.0
                    bl_path_quad[interp_id][2] = 0.0
                elif stage_id == 2:
                    # 维持home
                    fr_path_quad[interp_id][0] = 0.0
                    fr_path_quad[interp_id][1] = 0.0
                    fr_path_quad[interp_id][2] = 0.0
                    # home-amplitude到home+amplitude
                    bl_path_quad[interp_id][0] = -self.amplitudeX * cos(pi * tick_cnt / num_ticks) * 2.0
                    bl_path_quad[interp_id][1] = 0.0
                    bl_path_quad[interp_id][2] = abs(self.amplitudeZ) * sin(pi * tick_cnt / num_ticks) * -1.5
                elif stage_id == 3:
                    # 维持home
                    fr_path_quad[interp_id][0] = 0.0
                    fr_path_quad[interp_id][1] = 0.0
                    fr_path_quad[interp_id][2] = 0.0
                    # 维持home+amplitude
                    bl_path_quad[interp_id][0] = self.amplitudeX * 2.0
                    bl_path_quad[interp_id][1] = 0.0
                    bl_path_quad[interp_id][2] = 0.0
                elif stage_id == 4:
                    # home到home-amplitude
                    fr_path_quad[interp_id][0] = -self.amplitudeX * sin(pi / 2 * tick_cnt / num_ticks) * 2.0
                    fr_path_quad[interp_id][1] = 0.0
                    fr_path_quad[interp_id][2] = 0.0
                    # home+amplitude到home
                    bl_path_quad[interp_id][0] = self.amplitudeX * cos(pi / 2 * tick_cnt / num_ticks) * 2.0
                    bl_path_quad[interp_id][1] = 0.0
                    bl_path_quad[interp_id][2] = 0.0
                elif stage_id == 5:
                    # 维持home-amplitude
                    fr_path_quad[interp_id][0] = -self.amplitudeX * 2.0
                    fr_path_quad[interp_id][1] = 0.0
                    fr_path_quad[interp_id][2] = 0.0
                    # 维持home
                    bl_path_quad[interp_id][0] = 0.0
                    bl_path_quad[interp_id][1] = 0.0
                    bl_path_quad[interp_id][2] = 0.0
                else:
                    pass

        fl_path_quad = [[point[0], point[1], point[2]] for point in left_rotate_path(fr_path_quad, int(len(fr_path_quad) / 2))]
        br_path_quad = [[point[0], point[1], point[2]] for point in left_rotate_path(bl_path_quad, int(len(bl_path_quad) / 2))]

        path_quad_world = self.formated_path_status(fr_path_quad, br_path_quad, bl_path_quad, fl_path_quad, move_status)

        return path_quad_world







