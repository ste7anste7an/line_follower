from line_sensor import LineSensor
from pupremote import PUPRemoteSensor

pr = PUPRemoteSensor()
pr.add_channel('pos', 'bb')
ls = LineSensor()
ls.ir_led_on()
ls.rgb_led_mode(ls.INVERTED)

# # # Start calibration
# sensor.start_calibration()
# sleep(5)

ls.load_calibration_from_rom()
ls.mode_calibrated()

while 1:
    pos = ls.position()
    der = ls.position_derivative()
    print(pos, der) # Derivative is 124 always, so not useful.
    pr.update_channel('pos', pos, der)
    pr.process()