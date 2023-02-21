# import system modules
import traceback
import os
import sys
import time
import datetime
import screeninfo
# import utility modules
import numpy as np
import scipy.constants as const
import cv2
# import custom modules
import LidarControl
import SensorSignal


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
    lidar.reset_sensor()
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

    # display window setup
    cv2.namedWindow("Intensity", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    # move to target screen before full screen
    cv2.moveWindow("Intensity", monitors[0].x + 1, monitors[0].y + 1)
    cv2.setWindowProperty("Intensity", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    # main loop for LiDAR capturing
    while 1:

        # acquire physical sensor data
        sig = SensorSignal.acquire_signal(lidar, shift_vec=[0, 0], downsample_ratio=2)
        # progress info
        print(f" - Full frame captured.")

        # process LiDAR data
        distance = sig.calc_dist()
        # calculate avg intensity
        intensity = sig.calc_intensity()
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

    # end of while 1

# end of main program

except Exception as err:
    print("Error:", err)
    traceback.print_exc()

finally:
    # GC
    print("\n - GPIO clean up.")
    os.environ["DISPLAY"] = default_DISPLAY_env
    sys.exit(0)
