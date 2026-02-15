from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pupremote_hub import PUPRemoteHub

hub = PrimeHub()
lmotor = Motor(Port.F, Direction.COUNTERCLOCKWISE)
rmotor = Motor(Port.E)
pr = PUPRemoteHub(Port.C)
pr.add_channel('line', 'bbb')

FACTOR = 1.5
BASE_DC = 40 * FACTOR
KP = .65 * FACTOR
KD = 0.2 * FACTOR

while 1:
    pos,der,shape = pr.call('line')
    print(pos, der, chr(shape))
    if chr(shape) in '|<>T':
        lmotor.dc(BASE_DC-(abs(pos)*0.4+abs(der)*0.7) - pos*KP -der*KD)
        rmotor.dc(BASE_DC-(abs(pos)*0.4+abs(der)*0.7) + pos*KP +der*KD)
    else:
        lmotor.stop()
        rmotor.stop()



