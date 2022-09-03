# -*- coding: utf-8 -*-

# from micropython import const
from geometry import SIN10, COS10, SIN15, COS15, SIN30, COS30, SIN45, COS45
from utils import load_panel_html

# mounting position
leg_mount_x = 50  # position in x direction of the fore and hind legs
leg_mount_y = 50  # position in y direction of the fore and hind legs

# link length (Notes that the root coincides with the joint-1)
leg_joint1_2joint2 = 40
leg_joint2_2joint3 = 80
leg_joint3_2tip = 135.48

# PCA9685 Setting
pca_i2c_adr = 0x40
pca_i2c_scl = 22
pca_i2c_sda = 21
pca_i2c_freq = 100000

# servo specification
pulse_min = 500
pulse_max = 2500
pulse_freq = 50

# Pin definition
pin_voltage_monitor = 34
pin_red = 26
pin_green = 25
low_voltage = 10.2  # each of 3.4V

# WIFI config
wlan_sta = False
wifissid = 'LeeSophia'
wifipass = '******'

# calibration data saved path
calibration_path = "calibration.json"

# html
panel_html_dir = 'panel_http.html'
panel_html_str = load_panel_html(panel_html_dir)
calib_html_dir = 'calibration.html'
calib_html_str = load_panel_html(calib_html_dir)

# movement constance
standby_z = leg_joint3_2tip * COS10 - leg_joint2_2joint3 * SIN30
other_x = leg_mount_x + (leg_joint1_2joint2 + leg_joint2_2joint3 * COS30 + leg_joint3_2tip * SIN10) * COS45
other_y = leg_mount_y + (leg_joint1_2joint2 + leg_joint2_2joint3 * COS30 + leg_joint3_2tip * SIN10) * SIN45

# default (stand mode) local tip position
# frontal right
p1_x = other_x
p1_y = other_y
p1_z = standby_z
# back right
p2_x = -other_x
p2_y = other_y
p2_z = standby_z
# back left
p3_x = -other_x
p3_y = -other_y
p3_z = standby_z
# frontal left
p4_x = other_x
p4_y = -other_y
p4_z = standby_z


if __name__ == '__main__':
    print(other_x, other_y, standby_z)



