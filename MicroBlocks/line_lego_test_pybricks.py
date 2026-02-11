from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()



from micropup import MicroPUP
p=MicroPUP(Port.A)
p.add_command('pos',to_hub=2,from_hub=0)



while True:
    pos, shape = p.call('pos')   # pos expected ~0..255, center ~128
    print(pos,chr(shape))
  
    wait(10)
