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
import tkinter as tk
from PIL import ImageTk, Image
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
    display = monitors[0]
except:
    print("No display is attached!")
    os.environ["DISPLAY"] = default_DISPLAY_env
    sys.exit()
print()

# Lidar setup
lidar_cfg = LidarControl.LidarConfig()
lidar_cfg.width = int(104)  # not including header pixel
lidar_cfg.height = int(80)
lidar_cfg.Ndata = int(2)
lidar_cfg.Nlight = int(12000)
lidar_cfg.light_delay = 2.0
lidar = LidarControl.LidarControl(lidar_cfg)
try:
    lidar.connect_GPIO()
    lidar.connect_sensor()
    lidar.connect_MCU()
except Exception as err:
    print("Error:", err)
    os.environ["DISPLAY"] = default_DISPLAY_env
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
    os.environ["DISPLAY"] = default_DISPLAY_env
    sys.exit()
else:
    print(" - LiDAR ready.")


# main program
print()
try:

    # display window setup
    target_width = display.width // 2
    target_height = display.height // 2
    target_scale = min((target_width // lidar_cfg.width), (target_height // lidar_cfg.height))
    window_width = target_scale * lidar_cfg.width
    window_height = target_scale * lidar_cfg.height
    win_display = tk.Tk()
    win_display.geometry(f"{window_height}x{window_width}+{display.x}+{display.y}")
    panel = tk.Label(win_display)

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
        # make sure of no overflow values
        disp_intensity = np.minimum(intensity, 1024)
        disp_intensity = np.array(disp_intensity // 4, dtype=np.uint8)
        disp_intensity = np.rot90(disp_intensity, k=1, axes=(0,1))
        img = ImageTk.PhotoImage(Image.fromarray(disp_intensity).resize((window_height, window_width), Image.NEAREST))
        panel.configure(image=img)
        panel.pack()
        win_display.update()

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
