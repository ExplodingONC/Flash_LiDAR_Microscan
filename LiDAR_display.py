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
    Screen = monitors[0]
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
lidar_cfg.T0_pulse = int(8)
lidar_cfg.Light_pulse = int(7)
lidar_cfg.light_offset = 0.5
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

    # window settings
    img_rot = 1
    pack_side = tk.LEFT
    # decide window geometry
    target_width = int(Screen.width / 1.5)
    target_height = int(Screen.height / 1.5)
    if img_rot % 2 == 1:
        img_width = lidar_cfg.height
        img_height = lidar_cfg.width
    else:
        img_width = lidar_cfg.width
        img_height = lidar_cfg.height
    if pack_side == tk.LEFT or pack_side == tk.RIGHT:
        target_scale = min((target_width // img_width) // 2, (target_height // img_height))
        window_width = target_scale * img_width * 2
        window_height = target_scale * img_height
    else:
        target_scale = min((target_width // img_width), (target_height // img_height) // 2)
        window_width = target_scale * img_width
        window_height = target_scale * img_height * 2
    img_width = img_width * target_scale
    img_height = img_height * target_scale
    # display window setup
    win_display = tk.Tk()
    win_display.geometry(f"{window_width}x{window_height}+{Screen.x}+{Screen.y}")
    panel_intensity = tk.Label(win_display)
    panel_distance = tk.Label(win_display)

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
        print(f"Min: {np.min(distance):5.2f}  Max: {np.max(distance):5.2f}  Avg: {np.mean(distance):5.2f}")
        # center data
        sample_X = lidar_cfg.width // 2
        sample_Y = lidar_cfg.height // 2
        delta_F1 = int(sig.delta_F1[sample_Y, sample_X])
        delta_F2 = int(sig.delta_F2[sample_Y, sample_X])
        F1_Ch1 = sig.data[0, 0, sample_Y, sample_X] + sig.data[2, 1, sample_Y, sample_X]
        F1_Ch2 = sig.data[0, 1, sample_Y, sample_X] + sig.data[2, 0, sample_Y, sample_X]
        F2_Ch1 = sig.data[1, 0, sample_Y, sample_X] + sig.data[3, 1, sample_Y, sample_X]
        F2_Ch2 = sig.data[1, 1, sample_Y, sample_X] + sig.data[3, 0, sample_Y, sample_X]
        F1_avg = F1_Ch1 + F1_Ch2
        F2_avg = F2_Ch1 + F2_Ch2
        print("F1_Ch1", F1_Ch1, " F1_Ch2", F1_Ch2)
        print("F1_avg", F1_avg, "Delta_1", delta_F1)
        print("F2_Ch1", F2_Ch1, " F2_Ch2", F2_Ch2)
        print("F2_avg", F2_avg, "Delta_2", delta_F2)
        print(f"Dist {distance[sample_Y, sample_X]:5.2f}")
        # display intensity map
        # make sure of no overflow values
        disp_intensity = np.minimum(intensity, 1024)
        disp_intensity = np.array(disp_intensity // 4, dtype=np.uint8)
        disp_intensity = np.rot90(disp_intensity, k=img_rot, axes=(0, 1))
        img_I = ImageTk.PhotoImage(Image.fromarray(disp_intensity).resize((img_width, img_height), Image.NEAREST))
        panel_intensity.configure(image=img_I)
        panel_intensity.pack(side=pack_side)
        # display distance result
        disp_distance = np.minimum(distance, 5.0) * 50
        disp_distance = np.array(disp_distance, dtype=np.uint8)
        disp_distance = np.rot90(disp_distance, k=img_rot, axes=(0, 1))
        img_D = ImageTk.PhotoImage(Image.fromarray(disp_distance).resize((img_width, img_height), Image.NEAREST))
        panel_distance.configure(image=img_D)
        panel_distance.pack(side=pack_side)

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
