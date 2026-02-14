from machine import I2C, Pin
from time import sleep

class LineSensor:
    """
    MicroPython class for line following sensor via I2C.
    
    Reads 11 bytes:
    - Bytes 0-7: Light values from 8 sensors
    - Byte 8: Position
    - Byte 9: Min value
    - Byte 10: Max value
    """
    
    # Command constants
    MODE_RAW = 0
    MODE_CALIBRATED = 1
    CMD_GET_VERSION = 2
    CMD_DEBUG = 3
    CMD_CALIBRATE = 4
    CMD_IS_CALIBRATED = 5
    CMD_LOAD_CAL = 6 #load calibrated values frpom eeprom
    CMD_SAVE_CAL = 7 #save calibrated values to eeprom
    CMD_GET_MIN = 8
    CMD_GET_MAX = 9
    CMD_SET_MIN = 10
    CMD_SET_MAX = 11
    CMD_NEOPIXEL = 12 # neopixel: lednr, r, g, b, write
    CMD_LEDS = 13
    CMD_SET_EMITTER = 14 # optional for qtr sensors, 1 for on, 0 for zero.
    MAX_CMDS = 15

    # LED Modes
    LEDS_OFF = 0
    LEDS_NORMAL = 1
    LEDS_INVERTED = 2
    LEDS_POSITION = 3
    LEDS_MAX = 4

    def __init__(self, scl_pin=4, sda_pin=5, device_addr=51):
        """
        Initialize the line sensor.
        
        Args:
            scl_pin: SCL pin number (default 4)
            sda_pin: SDA pin number (default 5)
            device_addr: I2C device address (default 51)
            freq: I2C frequency in Hz (default 100000)
        """
        self.device_addr = device_addr
        self.i2c = I2C(1,scl=Pin(scl_pin), sda=Pin(sda_pin))
    
    def light_values(self):
        """
        Read only the 8 light sensor values.
        
        Returns:
            list: List of 8 light sensor values
        """
        data = self.i2c.readfrom(self.device_addr, 13)
        return list(data[0:8])

    def raw_data(self):
        return self.i2c.readfrom(self.device_addr, 13)
    
    def position(self):
        """
        Read the position value.
        
        Returns:
            int: Position value (byte 8)
        """
        data = self.i2c.readfrom(self.device_addr, 13)
        return data[8]-128

    def position_derivative(self):
        """
        Read the position value.
        
        Returns:
            int: Position value (byte 8)
        """
        data = self.i2c.readfrom(self.device_addr, 13)
        return data[11]-128
    
    def write_command(self, command):
        """
        Write a 1-byte command to the sensor.
        
        Args:
            command: Command byte to send (MODE_RAW, MODE_CALIBRATED, or START_CALIBRATION)
        """
        if type(command) is int:
            command = [command]
        self.i2c.writeto(self.device_addr, bytes(command))
    
    def mode_raw(self):
        """Set sensor to raw mode."""
        self.write_command(self.MODE_RAW)
    
    def mode_calibrated(self):
        """Set sensor to calibrated mode."""
        self.write_command(self.MODE_CALIBRATED)
    
    def start_calibration(self):
        """Start sensor calibration."""
        self.write_command(self.CMD_CALIBRATE)

    def ir_led_on(self):
        self.write_command((self.CMD_SET_EMITTER, 1))
        
    def ir_led_off(self):
        self.write_command((self.CMD_SET_EMITTER, 0))
        
    def rgb_led_mode(self, mode):
        self.write_command((self.CMD_LEDS, mode))
        
    def save_calibration_in_rom(self):
        self.write_command(self.CMD_SAVE_CAL)
        
    def load_calibration_from_rom(self):
        self.write_command(self.CMD_LOAD_CAL)
        


# Example usage:
if __name__ == "__main__":
    # Initialize sensor
    sensor = LineSensor()
    
    sensor.ir_led_on()
    sensor.rgb_led_mode(sensor.LEDS_INVERTED)
    
    # # # Start calibration
    # sensor.start_calibration()
    # sleep(5)
    
    sensor.load_calibration_from_rom()
    sensor.mode_calibrated()
    
    # Read just light values
    for i in range(1000):
        pos = sensor.position()
        der = sensor.position_derivative()
        print("Pos:", pos, der)
        sleep(0.1)