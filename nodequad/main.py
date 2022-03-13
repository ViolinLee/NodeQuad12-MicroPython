import _thread
from utime import sleep_ms
from machine import I2C, ADC, Pin, Timer
from network import WLAN, STA_IF
from nodequad import NodeQuad
from setting import *
from utils import battery_monitor_loop


if __name__ == '__main__':
    # Connect to WIFI
    wifi = WLAN(STA_IF)
    wifi.active(True)
    wifi.connect(wifissid, wifipass)
    while not wifi.isconnected():
        pass
    sta_ip = wifi.ifconfig()[0]
    print("Listening, connect your browser to %s" % sta_ip)

    # Quadruped Initialization
    iic = I2C(scl=Pin(pca_i2c_scl), sda=Pin(pca_i2c_sda), freq=pca_i2c_freq)
    quadruped = NodeQuad(sta_ip, iic, pca_i2c_adr, pulse_min, pulse_max, pulse_freq)
    quadruped.init()
    sleep_ms(500)
    
    # Battery monitor routine
    adc = ADC(Pin(pin_voltage_monitor))
    adc.atten(ADC.ATTN_11DB)  # range of 3.3v
    adc.width(ADC.WIDTH_12BIT)  # 12bit accuracy
    led = Pin(pin_red, Pin.OUT)
    timer = Timer(1)
    timer.init(period=1000, mode=Timer.PERIODIC, callback=lambda t: battery_monitor_loop(adc, led, low_voltage))
    
    # Listening and update remote controller status
    _thread.start_new_thread(quadruped.web_controller.loop, ())

    # PyDog mainloop
    quadruped.main_loop()
    print("************Welcome to NodeQuad!************")



