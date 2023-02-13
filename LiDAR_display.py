# import system modules
import errno
import os
import sys
import subprocess
import time
import datetime
import screeninfo
import threading
# import project modules
import ctypes
import numpy as np
import scipy.constants as const
import cv2
# import I/O modules
import RPi.GPIO as GPIO
import smbus2
import spidev


# parameters
width = int(104)  # not including header pixel
height = int(80)
Ndata = int(2)
pixel_count = (width + 1) * height

# timestamp
date = datetime.datetime.now().astimezone()
print(date.strftime("%Y-%m-%d %H:%M:%S.%f %Z %z"))
print()

# check default display count
default_DISPLAY_env = os.getenv("DISPLAY")
try:
    default_monitor_cnt = len(screeninfo.get_monitors())
except:
    default_monitor_cnt = 0

# check operating system and physical display
if "win" in sys.platform:
    # Windows
    physical_DISPLAY_env = "localhost:0.0"
elif "linux" in sys.platform:
    # Linux
    physical_DISPLAY_env = ":0"
else:
    print("Unsupported OS version!")  # aka MacOS which I don't have
    sys.exit()
os.environ["DISPLAY"] = physical_DISPLAY_env
try:
    physical_monitor_cnt = len(screeninfo.get_monitors())
except:
    physical_monitor_cnt = 0

# decide DISPLAY environment variable
print(f"{default_monitor_cnt} default monitors, {physical_monitor_cnt} physical monitors.")
if physical_monitor_cnt > 0:
    os.environ["DISPLAY"] = physical_DISPLAY_env
    print(f"Physical monitors available! Using physical displays.")
else:
    os.environ["DISPLAY"] = default_DISPLAY_env
    if default_monitor_cnt > 0:
        print(f"No physical monitors! Using default settings.")
    else:
        print(f"No monitors attached! Nowhere to display. Exiting...")
        sys.exit()

# double check if display is available and acquire final infos
try:
    monitors = screeninfo.get_monitors()
    print("Total screen count:", len(monitors))
    for monitor in monitors:
        print(monitor)
except:
    print("No display is attached!")
    sys.exit()
print()

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

# setup SPI bus
try:
    spi_channel = 0
    spi_device_MCU = 0
    spi = spidev.SpiDev()
    spi.open(spi_channel, spi_device_MCU)
except Exception as err:
    print("Error:", err)
    print("SPI initialization failed!")
    sys.exit()
else:
    spi.max_speed_hz = 5000000
    spi.mode = 0b11
    spi.bits_per_word = 8
    spi.lsbfirst = False
    print(f"SPI initialized at {spi.max_speed_hz}Hz.")

# setup GPIOs
try:
    # GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    pin_sensor_rst_P = 4
    pin_mcu_rst_N = 23
    GPIO.setup(pin_sensor_rst_P, GPIO.OUT, initial=0)  # sensor reset (P)
    # GPIO.setup(pin_mcu_rst_N, GPIO.OUT, initial=1)  # MCU reset (N)
except Exception as err:
    print("Error:", err)
    print("GPIO initialization failed!")
    sys.exit()
else:
    print("GPIO initialized.")

# load MCU binary
try:
    load_cmd = ["openocd",
                "-f", "interface/raspberrypi-swd.cfg",
                "-f", "target/rp2040.cfg",
                "-c", "program dvp2spi_lnk.elf verify reset exit"]
    subprocess.run(load_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
except Exception as err:
    print("Error:", err)
    print("Load binary failed!")
else:
    print("MCU binary loaded.")

# reset devices
time.sleep(0.1)
GPIO.output(pin_sensor_rst_P, 1)
# GPIO.output(pin_mcu_rst_N, 0)
time.sleep(0.1)
GPIO.output(pin_sensor_rst_P, 0)
# GPIO.output(pin_mcu_rst_N, 1)
time.sleep(0.1)
print("Devices reset.")

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
    (0x1F, T0_pulse_width//2),  # P4_delay
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

    # LiDAR setup through I2C
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
    print()
    time.sleep(0.25)

    # display window setup
    cv2.namedWindow("Intensity", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    # move to target screen before full screen
    cv2.moveWindow("Intensity", monitors[0].x + 1, monitors[0].y + 1)
    cv2.setWindowProperty("Intensity", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    # main loop for LiDAR capturing
    while 1:

        # [F1..F4] [VTX1,VTX2] [Y] [X]
        data = np.zeros((4, 2, height, width), dtype=np.int16)

        # 4 subframes F1..F4
        for subframe in range(1, 5):  # that's [1:4]
            # progress info
            print(f" - Trigger subframe F{subframe} capture and SPI read.")
            # command MCU to start frame capturing
            time.sleep(0.01) # wait for MCU to flush FIFO
            spi.writebytes([0x00 | subframe])
            # query frame state
            timeout_counter = 0
            while True:
                frame_state = spi.readbytes(1)
                if frame_state[0] == (0x10 | subframe):
                    time.sleep(0.01) # wait for MCU to flush FIFO
                    break
                else:
                    timeout_counter += 1
                    # re-trigger if there is a timeout (SPI command lost)
                    if (timeout_counter > 250):
                        timeout_counter = 0
                        spi.writebytes([0x00 | subframe])
                        print(f" - Re-trigger subframe F{subframe} capture.")
                    time.sleep(0.01)
            # data transfering
            data_stream = np.zeros((Ndata, height, 2 * (width + 1)), dtype=np.int16)
            for integr in range(0, Ndata):
                for line in range(0, height):
                    temp = spi.readbytes(4 * (width + 1))
                    temp = np.array(temp, dtype=np.int16)
                    data_stream[integr, line, :] = (temp[1::2] & 0x0f) << 8 | temp[0::2]
            data[subframe - 1, 0, :, :] = data_stream[1, :, 2::2] - data_stream[0, :, 2::2]
            data[subframe - 1, 1, :, :] = data_stream[1, :, 3::2] - data_stream[0, :, 3::2]
        # end of for subframe in range(1, 5)

        # progress info
        print(f" - Full frame captured.")
        # make sure of no negative values
        data = np.maximum(data, 0)
        # delta_F1 = (F1_Ch2 - F1_Ch1) + (F3_Ch1 - F3_Ch2)
        delta_F1 = (data[0, 1, :, :] - data[0, 0, :, :]) \
                 + (data[2, 0, :, :] - data[2, 1, :, :])
        # delta_F2 = (F2_Ch2 - F2_Ch1) + (F4_Ch1 - F4_Ch2)
        delta_F2 = (data[1, 1, :, :] - data[1, 0, :, :]) \
                 + (data[3, 0, :, :] - data[3, 1, :, :])
        # calculate avg intensity
        intensity = np.sum(data, axis=(0, 1)) // 8
        # calculate distance
        distance = (delta_F1 >= 0) \
                    * (delta_F2 / (np.abs(delta_F1) + np.abs(delta_F2)) + 1) \
                    * (const.speed_of_light * T0_pulse_time) / 4 \
                 + (delta_F1 < 0) \
                    * (-delta_F2 / (np.abs(delta_F1) + np.abs(delta_F2)) + 3) \
                    * (const.speed_of_light * T0_pulse_time) / 4
        # print distance
        np.set_printoptions(formatter={'float': lambda x: "{0:5.2f}".format(x)})
        print(distance)
        # display intensity map
        disp_intensity = np.array(intensity // 4, dtype=np.uint8)
        # make sure of no overflow values
        disp_intensity = np.minimum(disp_intensity, 255)
        cv2.imshow("Intensity", disp_intensity)
        cv2.waitKey(1)
        print()
        # pause a bit
        # time.sleep(1)

    # end of while 1

# end of main program

except Exception as err:
    print("Error:", err)

finally:
    # GC
    GPIO.cleanup()
    spi.close()
    print("\n - GPIO clean up.")
    os.environ["DISPLAY"] = default_DISPLAY_env
    sys.exit(0)
