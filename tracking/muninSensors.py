from smbus2 import SMBus
import time

class MuninGPS :
    def __init__(self, i2c_bus: int, i2c_addr,):
        self.init = False
        self.bus_numer = i2c_bus
        self.addr = i2c_addr
        self.bus:(SMBus|None) = None
        self.start()
    
    def start(self):
        try:
            self.bus = SMBus(self.bus_numer)
            time.sleep(0.3)
        except Exception:
            print(f"Failed to start i2c bus{self.bus_numer}", )

    def readNmea(self):
        try:
            high_byte = self.bus.read_i2c_block_data(self.addr, 0xFD,1)
            low_byte = self.bus.read_i2c_block_data(self.addr, 0xFE,1)

            bytes_availabel = high_byte << 8 | low_byte

            if bytes_availabel > 0:
                data = self.bus.read_i2c_block_data(gps_i2c_addr, 0xFF, min(bytes_availabel, 32))
                nmea = bytes(data).decode(encoding=['utf-8'], errors='ignore')
                print(nmea)
        except Exception:
            print(Exception)
