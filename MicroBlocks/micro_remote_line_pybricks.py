from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

from microremote import MicroRemote

ur=MicroRemote(Port.A)

wait(1000)
#ur.call('lineon')
while True:
    err,data=ur.call('linepos')
    pos,shape = data
    print(data)
    nr = (pos+128)*5/255
    hub.display.off()
    hub.display.pixel(3,nr,100)
    #wait(100)

