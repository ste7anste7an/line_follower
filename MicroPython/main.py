from line_sensor import LineSensor
from pupremote import PUPRemoteSensor
from time import sleep

pr = PUPRemoteSensor()
pr.add_channel('line', 'bbb')
pr.process()

ls = LineSensor()

ls.ir_led_on()

ls.rgb_led_mode(ls.LEDS_INVERTED)

# # # Start calibration
# Do this only once, and save the calibration to ROM. Then you can load it from ROM in the future without needing to recalibrate.
# ls.start_calibration()
# sleep(5)
# ls.mode_calibrated()
# ls.save_calibration_in_rom()

ls.load_calibration_from_rom()
ls.mode_calibrated()

while 1:
    pos, der, shape = ls.data(ls.POSITION, ls.DERIVATIVE, ls.SHAPE)
    print(pos, der, chr(shape))
    pr.update_channel('line', pos, der, shape)
    pr.process()