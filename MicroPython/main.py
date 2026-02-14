from line_sensor import LineSensor
from pupremote import PUPRemoteSensor

pr = PUPRemoteSensor()
pr.add_channel('line', 'bb')

ls = LineSensor()

ls.ir_led_on()

ls.rgb_led_mode(ls.LEDS_INVERTED)

# # # Start calibration
# Do this only once, and save the calibration to ROM. Then you can load it from ROM in the future without needing to recalibrate.
# sensor.start_calibration()
# sleep(5)

ls.load_calibration_from_rom()
ls.mode_calibrated()

while 1:
    pos = ls.position()
    der = ls.position_derivative()
    print(pos, der)
    pr.update_channel('line', pos, der)
    pr.process()