# import system modules
import sys
import time
import datetime
# import utility modules
import numpy
# import custom modules
import LidarControl
import SensorSignal


# timestamp
date = datetime.datetime.now().astimezone()
print(date.strftime("%Y-%m-%d %H:%M:%S.%f %Z %z"))

# Lidar setup
lidar_cfg = LidarControl.LidarConfig()
lidar_cfg.width = int(104)  # not including header pixel
lidar_cfg.height = int(80)
lidar_cfg.Ndata = int(2)
lidar_cfg.Nlight = int(32000)
lidar_cfg.T0_pulse = int(8)
lidar_cfg.Light_pulse = int(7)
lidar_cfg.light_delay = 2.0
lidar = LidarControl.LidarControl(lidar_cfg)
try:
    lidar.connect_GPIO()
    lidar.connect_sensor()
    lidar.connect_MCU()
except Exception as err:
    print("Error:", err)
    sys.exit()
else:
    print(" - LiDAR hardware connected.")

# Lidar init
try:
    lidar.reset_device()
    lidar.load_MCU()
    lidar.setup_sensor()
except Exception as err:
    print("Error:", err)
    sys.exit()
else:
    print(" - LiDAR ready.")


# main program
print()
try:

    # main loop for LiDAR capturing
    while 1:

        # acquire physical sensor data
        data = lidar.acquire_data()
        sample_X = lidar_cfg.width // 2
        sample_Y = lidar_cfg.height // 2
        # progress info
        print(f" - Full frame captured.")
        delta_F1 = (data[0, 1, :, :] - data[0, 0, :, :]) \
            + (data[2, 0, :, :] - data[2, 1, :, :])
        delta_F2 = (data[1, 1, :, :] - data[1, 0, :, :]) \
            + (data[3, 0, :, :] - data[3, 1, :, :])
        F1_Ch1 = data[0, 0, sample_Y, sample_X] + data[2, 1, sample_Y, sample_X]
        F1_Ch2 = data[0, 1, sample_Y, sample_X] + data[2, 0, sample_Y, sample_X]
        F2_Ch1 = data[1, 0, sample_Y, sample_X] + data[3, 1, sample_Y, sample_X]
        F2_Ch2 = data[1, 1, sample_Y, sample_X] + data[3, 0, sample_Y, sample_X]
        F1_avg = F1_Ch1 + F1_Ch2
        F2_avg = F2_Ch1 + F2_Ch2
        print("F1_Ch1", F1_Ch1, " F1_Ch2", F1_Ch2)
        print("F1_avg", F1_avg, "Delta_1", delta_F1[sample_Y, sample_X])
        print("F2_Ch1", F2_Ch1, " F2_Ch2", F2_Ch2)
        print("F2_avg", F2_avg, "Delta_2", delta_F2[sample_Y, sample_X])

        # calculate avg intensity
        intensity = numpy.sum(data, axis=(0, 1)) // 8
        # print(intensity)
        # pause a bit
        print()
        time.sleep(1)
    # end of while 1

except Exception as err:
    print("Error:", err)

finally:
    # GC
    print("\n - GPIO clean up.")
    sys.exit(0)
