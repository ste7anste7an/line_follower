from machine import I2C, Pin
from time import sleep_ms
i2c = I2C(0,sda=Pin(5),scl=Pin(4))

def print_sensor(buf):
    for b in buf:
        print("*"*((255-b)//25))


while(1):
    buf =  i2c.readfrom(51,8)
    print_sensor(buf)
    print()
    sleep_ms(200)   