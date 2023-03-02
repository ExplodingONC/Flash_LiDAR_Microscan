# import system modules
import os
import sys
import errno
import subprocess
import time
import signal
import functools

# import I/O modules
import RPi.GPIO as GPIO
import smbus2
import spidev
# import utility modules
import math
import numpy as np
import scipy.constants as const
from dataclasses import dataclass


def timeout(seconds=10, error_message=os.strerror(errno.ETIME)):
    def decorator(func):
        def _handle_timeout(signum, frame):
            raise TimeoutError(error_message)

        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            signal.signal(signal.SIGALRM, _handle_timeout)
            signal.alarm(seconds)
            try:
                result = func(*args, **kwargs)
            finally:
                signal.alarm(0)
            return result
        return wrapper
    return decorator


@dataclass
class LidarConfig:
    # dimensions
    height: int = 80
    width: int = 104
    Ndata: int = 2
    Nlight: int = 1024
    # timing
    T0_pulse: int = 8
    VTX3_pulse: int = 28
    ADC_delay: int = 1
    light_delay: float = 1.5
    extrst_pulse: int = 16384
    frame_blank: int = 255

    def generate_reg_map(self):
        light_delay_int = math.ceil(self.light_delay)
        light_delay_half = self.light_delay % 1 > 0
        return (
            (0x00, 0b11100011),  # stop operation
            (0x07, 0b11000000),  # unknown?
            (0x08, (self.extrst_pulse >> 8) & 0xFF),  # ext_reset
            (0x09, (self.extrst_pulse) & 0xFF),
            (0x0A, (self.width >> 8) & 0xFF),  # H_pixel_num
            (0x0B, (self.width) & 0xFF),
            (0x0C, (self.height >> 8) & 0xFF),  # V_pixel_num
            (0x0D, (self.height) & 0xFF),
            (0x0E, 0x25),  # HST_offset
            (0x0F, 0b10110111),  # light_pattern
            (0x10, (self.frame_blank) & 0xFF),  # frame blanking
            (0x11, (self.frame_blank >> 8) & 0xFF),
            (0x12, (self.ADC_delay) & 0x1F),  # ADC_delay_cfg
            (0x13, (0b0100 << 4) | ((self.Nlight >> 16) & 0x0F)),  # LV_delay, Nlight
            (0x14, (self.Nlight >> 8) & 0xFF),
            (0x15, (self.Nlight) & 0xFF),
            (0x16, (self.Ndata) & 0xFF),  # Ndata (must be >1, otherwise only reset value is read and VTX won't trigger)
            (0x17, (self.T0_pulse) & 0xFF),  # VTX1
            (0x18, (self.T0_pulse) & 0xFF),  # VTX2
            (0x19, (self.VTX3_pulse >> 8) & 0xFF),  # VTX3
            (0x1A, (self.VTX3_pulse) & 0xFF),
            (0x1B, (self.T0_pulse) & 0xFF),  # light_pulse_width
            (0x1D, (light_delay_int) & 0xFF),  # light_pulse_offset
            (0x1F, (self.T0_pulse >> 1) & 0x7F),  # P4_half_delay, P4_delay
            (0x20, (0b0 << 7) | ((light_delay_half << 6) & 0x40) | (0b1001)),  # L/A, Light_pulse_half_delay, H_pixel_blanking
            # (0x21, 0x00),  # T1 (linear only)
            # (0x22, 0x00),  # PHIS (linear only)
            # (0x23, 0x00),  # T2 (linear only)
            (0x24, 0b00001111),  # timing signal enable: light/VTX1/VTX2/VTX3
            (0x00, 0b11000011),  # start clock divider
            (0x00, 0b10000011),  # start clock
            (0x00, 0b00000011),  # start timing gen
        )


class LidarControl:

    # physical
    width: int = int(104)  # not including header pixel
    height: int = int(80)
    Ndata: int = int(2)
    T_0: float = 8 / 60 * 1e-6
    # I/O
    i2c_dev = []
    i2c_channel = 1
    i2c_address_lidar = 0x2A
    spi_dev = []
    spi_channel = 0
    spi_device_MCU = 0
    pin_sensor_rst_P = 4
    pin_mcu_rst_N = 23

    def __init__(self, config=LidarConfig()):
        self.width = config.width
        self.height = config.height
        self.Ndata = config.Ndata
        self.T_0 = config.T0_pulse / 60 * 1e-6
        self.config = config

    def __del__(self):
        print("LiDAR clean up called.")
        try:
            self.spi_dev.close()
            GPIO.cleanup()
        except Exception as err:
            print("Fail to clean GPIO.")
            print(err)

    def connect_GPIO(self):
        GPIO.setmode(GPIO.BCM)

    def connect_sensor(self, i2c_ch=1, i2c_addr=0x2A, pin_sensor_rst=4):
        self.i2c_channel = i2c_ch
        self.i2c_address_lidar = i2c_addr
        self.pin_sensor_rst_P = pin_sensor_rst
        try:
            GPIO.setup(self.pin_sensor_rst_P, GPIO.OUT, initial=0)  # sensor reset (P)
        except Exception as err:
            print("Error:", err)
            print("Sensor rst pin initialization failed!")
            raise RuntimeError("GPIO not available!")
        else:
            print("Sensor rst pin initialized.")
        try:
            i2c_sensor = smbus2.SMBus(self.i2c_channel)
        except FileNotFoundError as err:
            print("FileNotFoundError", err)
            if err.errno == 2:
                print("I2C not enabled. Check raspi-config.")
            raise RuntimeError("I2C bus not available!")
        except Exception as err:
            print("Error:", err)
            print("I2C initialization failed!")
            raise RuntimeError("I2C bus init failed!")
        else:
            print("I2C initialized.")
            self.i2c_dev = i2c_sensor

    def connect_MCU(self, spi_ch=0, spi_num=0, pin_mcu_rst=23):
        self.spi_channel = spi_ch
        self.spi_device_MCU = spi_num
        self.pin_mcu_rst_N = pin_mcu_rst
        try:
            GPIO.setup(self.pin_mcu_rst_N, GPIO.OUT, initial=1)  # MCU reset (N)
        except Exception as err:
            print("Error:", err)
            print("MCU rst pin initialization failed!")
            raise RuntimeError("GPIO not available!")
        else:
            print("MCU rst pin initialized.")
        try:
            spi_mcu = spidev.SpiDev()
            spi_mcu.open(self.spi_channel, self.spi_device_MCU)
        except Exception as err:
            print("Error:", err)
            print("SPI initialization failed!")
            raise RuntimeError("SPI bus init failed!")
        else:
            spi_mcu.max_speed_hz = 5000000
            spi_mcu.mode = 0b11
            spi_mcu.bits_per_word = 8
            spi_mcu.lsbfirst = False
            print(f"SPI initialized at {spi_mcu.max_speed_hz}Hz.")
            self.spi_dev = spi_mcu

    def reset_device(self, sensor=True, mcu=False):
        if sensor:
            GPIO.output(self.pin_sensor_rst_P, 1)
        if mcu:
            GPIO.output(self.pin_mcu_rst_N, 0)
        time.sleep(0.05)
        if sensor:
            GPIO.output(self.pin_sensor_rst_P, 0)
        if mcu:
            GPIO.output(self.pin_mcu_rst_N, 1)
        time.sleep(0.05)
        print("Devices reset.")

    @timeout(5)
    def load_MCU(self, binary_path="dvp2spi_lnk.elf"):
        try:
            load_cmd = ["openocd",
                        "-f", "interface/raspberrypi-swd.cfg",
                        "-f", "target/rp2040.cfg",
                        "-c", f"program {binary_path} verify reset exit"]
            subprocess.run(load_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        except Exception as err:
            print("Error:", err)
            print("Load binary failed!")
            raise RuntimeError("MCU binary loading failed!")
        else:
            print("MCU binary loaded.")
            return 0

    def setup_sensor(self):
        try:
            lidar_reg_map = self.config.generate_reg_map()
            # write regs and check step-by-step
            for instruction in lidar_reg_map:
                self.i2c_dev.write_byte_data(self.i2c_address_lidar, instruction[0], instruction[1])
                if instruction[1] != self.i2c_dev.read_byte_data(self.i2c_address_lidar, instruction[0]):
                    raise Exception(f"Register validation failed! @{instruction}")
        except OSError as err:
            print("OSError", err)
            if err.errno == 121:
                print("I2C: No response from device! Check wiring on GPIO2/3.")
            raise RuntimeError("I2C device not connected!")
        except Exception as err:
            print("Error:", err)
            print("I2C unknown error!")
            raise RuntimeError("I2C unknown error!")
        else:
            print("I2C data sent.")
            time.sleep(0.05)

    @timeout(20)
    def acquire_data(self):
        # [F1..F4] [VTX1,VTX2] [Y] [X]
        data = np.zeros((4, 2, self.height, self.width), dtype=np.int16)
        # 4 subframes F1..F4
        for subframe in range(1, 5):  # that's [1:4]
            # progress info
            print(f" - Trigger subframe F{subframe} capture and SPI read.")
            # command MCU to start frame capturing
            time.sleep(0.01)  # wait for MCU to flush FIFO
            self.spi_dev.writebytes([0x00 | subframe])
            # query frame state
            timeout_counter = 0
            while True:
                frame_state = self.spi_dev.readbytes(1)
                if frame_state[0] == (0x10 | subframe):
                    time.sleep(0.01)  # wait for MCU to flush FIFO
                    break
                else:
                    timeout_counter += 1
                    # re-trigger if there is a timeout (SPI command lost)
                    if (timeout_counter > 250):
                        timeout_counter = 0
                        self.spi_dev.writebytes([0x00 | subframe])
                        print(f" - Re-trigger subframe F{subframe} capture.")
                    time.sleep(0.01)
            # data transfering
            data_stream = np.zeros((self.Ndata, self.height, 2 * (self.width + 1)), dtype=np.int16)
            for integr in range(0, self.Ndata):
                for line in range(0, self.height):
                    temp = self.spi_dev.readbytes(4 * (self.width + 1))
                    temp = np.array(temp, dtype=np.int16)
                    data_stream[integr, line, :] = (temp[1::2] & 0x0f) << 8 | temp[0::2]
            data[subframe - 1, 0, :, :] = data_stream[self.Ndata - 1, :, 2::2] - data_stream[0, :, 2::2]
            data[subframe - 1, 1, :, :] = data_stream[self.Ndata - 1, :, 3::2] - data_stream[0, :, 3::2]
        # end of for subframe in range(1, 5)
        data = np.maximum(data, 0)  # make sure of no negative values
        return data
