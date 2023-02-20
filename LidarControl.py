# import system modules
import os
import sys
import subprocess
import time
# import I/O modules
import RPi.GPIO as GPIO
import smbus2
import spidev
# import utility modules
import numpy as np
import scipy.constants as const


# LiDAR register map
T0_pulse_width = 0x08
T0_pulse_time = T0_pulse_width / 60 * 1e-6
lidar_reg_map = (
    (0x00, 0b11100011),  # stop operation
    (0x07, 0b11000000),  # unknown?
    (0x08, 0x40),  # ext_reset
    (0x09, 0x00),
    (0x0A, 0x00),  # H_pixel_num
    (0x0B, 0x68),
    (0x0C, 0x00),  # V_pixel_num
    (0x0D, 0x50),
    (0x0E, 0x25),  # HST_offset
    (0x0F, 0b10110111),  # light_pattern
    (0x10, 0xFF),  # frame blanking
    (0x11, 0x00),
    (0x12, 0x08),  # ADC_delay_cfg
    (0x13, 0b01000000),  # LV_delay, Nlight
    (0x14, 0x04),
    (0x15, 0x00),
    (0x16, 0x02),  # Ndata (must be >1, otherwise only reset value is read and VTX won't trigger)
    (0x17, T0_pulse_width),  # VTX1
    (0x18, T0_pulse_width),  # VTX2
    (0x19, 0x00),  # VTX3
    (0x1A, 0x1C),
    (0x1B, T0_pulse_width),  # light_pulse_width
    (0x1D, 0x01),  # light_pulse_offset
    (0x1F, T0_pulse_width // 2),  # P4_delay
    (0x20, 0b00001001),  # L/A, Light_pulse_half_delay, H_pixel_blanking
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
    width = int(104)  # not including header pixel
    height = int(80)
    Ndata = int(2)
    pixel_count = (width + 1) * height
    # I/O
    i2c_dev = []
    i2c_channel = 1
    i2c_address_lidar = 0x2A
    spi_dev = []
    spi_channel = 0
    spi_device_MCU = 0
    pin_sensor_rst_P = 4
    pin_mcu_rst_N = 23

    def __init__(self, res=[104, 80], Ndata=2):
        self.width, self.height = res
        self.Ndata = Ndata
        self.pixel_count = (self.width + 1) * self.height

    def connect_GPIO(self, sensor_rst=4, mcu_rst=23):
        self.pin_sensor_rst_P = sensor_rst
        self.pin_mcu_rst_N = mcu_rst
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pin_sensor_rst_P, GPIO.OUT, initial=0)  # sensor reset (P)
            # GPIO.setup(pin_mcu_rst_N, GPIO.OUT, initial=1)  # MCU reset (N)
        except Exception as err:
            print("Error:", err)
            print("GPIO initialization failed!")
            raise RuntimeError("GPIO not available!")
        else:
            print("GPIO initialized.")

    def connect_sensor(self, i2c_ch=1, i2c_addr=0x2A):
        self.i2c_channel = i2c_ch
        self.i2c_address_lidar = i2c_addr
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
        
    def connect_MCU(self, spi_ch=0, spi_num=0):
        self.spi_channel = spi_ch
        self.spi_device_MCU = spi_num
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
        
    def reset_sensor(self):
        GPIO.output(self.pin_sensor_rst_P, 1)
        time.sleep(0.05)
        GPIO.output(self.pin_sensor_rst_P, 0)
        time.sleep(0.05)
        print("Devices reset.")

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
            # write regs and check step-by-step
            for instruction in lidar_reg_map:
                self.i2c_dev.write_byte_data(self.i2c_address_lidar, instruction[0], instruction[1])
                if instruction[1] != self.i2c_dev.read_byte_data(self.i2c_address_lidar, instruction[0]):
                    raise Exception(f"Register validation failed! @{instruction}")
            time.sleep(0.01)
        except OSError as err:
            print(" -", "OSError", err)
            if err.errno == 121:
                print(" - I2C: No response from device! Check wiring on GPIO2/3.")
            raise RuntimeError("I2C device not connected!")
        except Exception as err:
            print("Error:", err)
            print(" - I2C unknown error!")
            raise RuntimeError("I2C unknown error!")
        else:
            print(" - I2C data sent.")
            time.sleep(0.05)

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

