def load_panel_html(html_dir):
    with open(html_dir, encoding='utf-8') as f:
        panel_html = f.read()
    return panel_html
    
    
def battery_monitor_loop(adc_pin, alarm_led, low_voltage=10.2):
    """a 3S LiPo battery (fully charged = 12.6V, nominal = 11.4V, discharged = 10.2V)"""
    # 4 for shunt resistance & 0.12V for offset compensate
    batt_voltage = 4 * (sum([adc_pin.read() / 4095 * 3.3 for i in range(100)]) / 100 - 0.12)
    if batt_voltage <= low_voltage:
        alarm_led.value(1)
    else:
        pass

