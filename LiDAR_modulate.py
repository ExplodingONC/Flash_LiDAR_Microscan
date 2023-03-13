# import system modules
import traceback
import os
import sys
import subprocess
import time
import datetime
import screeninfo
# import utility modules
import numpy as np
import scipy.constants as const
import tkinter as tk
from PIL import ImageTk, Image
# import custom modules
import gratings
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

# force physical DISPLAY environment
print(f"{default_monitor_cnt} default monitors, {physical_monitor_cnt} physical monitors.")
if physical_monitor_cnt > 0:
    os.environ["DISPLAY"] = physical_DISPLAY_env
    print(f"Physical monitors available! Using physical displays.")
else:
    os.environ["DISPLAY"] = default_DISPLAY_env
    print(f"No physical monitors reqiured for modulating! Exiting...")
    sys.exit() # commented for debug

# double check if display is available and acquire final infos
try:
    monitors = screeninfo.get_monitors()
    print("Total screen count:", len(monitors))
    for monitor in monitors:
        print(monitor)
    LCoS = monitors[0]
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
lidar_cfg.light_delay = 2
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

    # modulation window setup
    win_modulation = tk.Tk()
    win_modulation.geometry(f"{LCoS.width}x{LCoS.height}+{LCoS.x}+{LCoS.y}")
    win_modulation.config(cursor="none")  # hide cursor
    win_modulation.overrideredirect(True)  # borderless
    win_modulation.attributes('-fullscreen', True)  # full screen
    panel_modulate = tk.Label(win_modulation)

    # light path modulation images
    modulation = np.zeros([5, LCoS.height, LCoS.width], dtype=np.uint8)
    modulation[0, :, :] = gratings.blazed_grating([LCoS.width, LCoS.height], pitch=[np.inf, np.inf])
    modulation[1, :, :] = gratings.blazed_grating([LCoS.width, LCoS.height], pitch=[20, 20])
    modulation[2, :, :] = gratings.blazed_grating([LCoS.width, LCoS.height], pitch=[20, -20])
    modulation[3, :, :] = gratings.blazed_grating([LCoS.width, LCoS.height], pitch=[-20, 20])
    modulation[4, :, :] = gratings.blazed_grating([LCoS.width, LCoS.height], pitch=[-20, -20])

    # main loop for LiDAR capturing
    sigs = []
    for multi_frame in range(5):

        # LCoS modulation
        shift_vec = np.array([(multi_frame - 1) % 2, (multi_frame - 1) // 2]) / 2
        img =  ImageTk.PhotoImage(Image.fromarray(modulation[multi_frame, :, :]))
        panel_modulate.configure(image=img)
        panel_modulate.pack()
        win_modulation.update()
        time.sleep(0.1)

        # acquire physical sensor data
        sig = SensorSignal.acquire_signal(lidar, shift_vec=shift_vec, downsample_ratio=2)
        sigs.append(sig)

        # progress info
        print(f" - Full frame {multi_frame} captured.")

        print()

    # end of for multi_frame in range(5)
    win_modulation.destroy()

    # display results
    os.environ["DISPLAY"] = default_DISPLAY_env
    # display window setup
    win_display = tk.Tk()
    panel_display = []
    imgs = []
    for i in range(5):
        panel_display.append(tk.Label(win_display))
        imgs.append(ImageTk.PhotoImage(Image.fromarray(sigs[i].calc_intensity())))
        panel_display[-1].configure(image=imgs[-1])
        panel_display[-1].pack()
    print(f" - Result displayd.")
    win_display.mainloop()

# end of main program

except Exception as err:
    print("Error:", err)
    traceback.print_exc()

finally:
    # GC
    print("\n - GPIO clean up.")
    os.environ["DISPLAY"] = default_DISPLAY_env
    sys.exit(0)
