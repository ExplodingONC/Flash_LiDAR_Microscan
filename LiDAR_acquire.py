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
        # progress info
        print(f" - Full frame captured.")
        
        # calculate avg intensity
        intensity = numpy.sum(data, axis=(0,1)) // 8
        print(intensity)
        # pause a bit
        print()
        time.sleep(5)
    # end of while 1

except Exception as err:
    print("Error:", err)

finally:
    # GC
    print("\n - GPIO clean up.")
    sys.exit(0)
