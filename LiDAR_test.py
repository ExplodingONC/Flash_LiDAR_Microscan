# import system modules
import errno
import os
import sys
import time
import datetime
# import project modules
import ctypes
import numpy
import scipy.constants
# import I/O modules
import RPi.GPIO as GPIO
import smbus2


# parameters
width = int(104)  # not including header pixel
height = int(80)
Ndata = int(2)
pixel_count = (width + 1) * height

# timestamp
date = datetime.datetime.now().astimezone()
print(date.strftime("%Y-%m-%d %H:%M:%S.%f %Z %z"))

# setup IIC bus
try:
    i2c_channel = 1
    i2c_address_lidar = 0x2A
    i2c1 = smbus2.SMBus(i2c_channel)
except FileNotFoundError as err:
    print("FileNotFoundError", err)
    if err.errno == 2:
        print("I2C not enabled. Check raspi-config.")
    sys.exit()
except Exception as err:
    print("Error:", err)
    print("I2C initialization failed!")
    sys.exit()
else:
    print("I2C initialized.")

# setup GPIOs
try:
    # GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    pin_sensor_rst_P = 4
    pin_mcu_rst_N = 23
    GPIO.setup(pin_sensor_rst_P, GPIO.OUT)  # sensor reset (P)
    GPIO.setup(pin_mcu_rst_N, GPIO.OUT)  # MCU reset (N)
except Exception as err:
    print("Error:", err)
    print("GPIO initialization failed!")
    sys.exit()
else:
    print("GPIO initialized.")

# reset devices
time.sleep(0.1)
GPIO.output(pin_sensor_rst_P, 1)
time.sleep(0.1)
GPIO.output(pin_sensor_rst_P, 0)
time.sleep(0.1)
print("Devices reset.")

# LiDAR register map
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
    (0x17, 0x08),  # VTX1
    (0x18, 0x08),  # VTX2
    (0x19, 0x00),  # VTX3
    (0x1A, 0x01),
    (0x1B, 0x08),  # light_pulse_width
    (0x1D, 0x01),  # light_pulse_offset
    (0x1F, 0x04),  # P4_delay
    (0x20, 0b00001001),  # L/A, Light_pulse_half_delay, H_pixel_blanking
    # (0x21, 0x00),  # T1 (linear only)
    # (0x22, 0x00),  # PHIS (linear only)
    # (0x23, 0x00),  # T2 (linear only)
    (0x24, 0b00001111),  # timing signal enable: light/VTX1/VTX2/VTX3
    (0x00, 0b11000011),  # start clock divider
    (0x00, 0b10000011),  # start clock
    (0x00, 0b00000011),  # start timing gen
)

# main program
print()
try:

    # LiDAR setup through IIC
    try:
        # write regs and check step-by-step
        for instruction in lidar_reg_map:
            i2c1.write_byte_data(i2c_address_lidar, instruction[0], instruction[1])
            if instruction[1] != i2c1.read_byte_data(i2c_address_lidar, instruction[0]):
                raise Exception(f"Register validation failed! @{instruction}")
        time.sleep(0.01)
    except OSError as err:
        print(" -", "OSError", err)
        if err.errno == 121:
            print(" - I2C: No response from device! Check wiring on GPIO2/3.")
        sys.exit()
    except Exception as err:
        print("Error:", err)
        print(" - I2C unknown error!")
        sys.exit()
    else:
        print(" - I2C data sent.")
    time.sleep(0.25)

    while 1:
        # just let it free running
        pass
    
except Exception as err:
    print("Error:", err)

finally:
    # GC
    GPIO.cleanup()
    print("\n - GPIO clean up.")
    sys.exit(0)
