#!/usr/bin/env python3
from .i2c import I2C

class ADC(I2C):
    ADDR = 0x14                   # Address of the expansion board is 0x14

    def __init__(self, chn):       # Constructor with a parameter 'chn' representing the channel number
        super().__init__()

        # Check if the channel is specified as a string starting with "A"
        if isinstance(chn, str):
            if chn.startswith("A"):
                chn = int(chn[1:])   # If it starts with "A", extract the digit after "A"
            else:
                raise ValueError("ADC channel should be between [A0, A7], not {0}".format(chn))

        # Check if the channel is within the range of 0 to 7
        if chn < 0 or chn > 7:
            self._error('Incorrect channel range')

        chn = 7 - chn
        self.chn = chn | 0x10       # Set the channel for the slave address
        self.reg = 0x40 + self.chn  # Set the register value
        # self.bus = smbus.SMBus(1)

    def read(self):                # Method to read ADC channel data
        self._debug("Write 0x%02X to 0x%02X" % (self.chn, self.ADDR))
        # self.bus.write_byte(self.ADDR, self.chn)      # Write data to the bus
        self.send([self.chn, 0, 0], self.ADDR)

        self._debug("Read from 0x%02X" % (self.ADDR))
        # value_h = self.bus.read_byte(self.ADDR)
        value_h = self.recv(1, self.ADDR)[0]            # Read data from the bus

        self._debug("Read from 0x%02X" % (self.ADDR))
        # value_l = self.bus.read_byte(self.ADDR)
        value_l = self.recv(1, self.ADDR)[0]            # Read data again (read twice)

        value = (value_h << 8) + value_l
        self._debug("Read value: %s" % value)
        return value

    def read_voltage(self):        # Method to convert the read data to voltage (0~3.3V)
        return self.read * 3.3 / 4095


def test():
    import time
    adc0 = ADC(0)
    adc1 = ADC(1)
    adc2 = ADC(2)
    while True:
        print(adc0.read(), adc1.read(), adc2.read())
        time.sleep(0.25)

if __name__ == '__main__':
    test()
