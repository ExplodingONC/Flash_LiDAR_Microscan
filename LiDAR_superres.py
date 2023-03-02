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
import matplotlib.pyplot as plt
import matplotlib as mpl
# import custom modules
import gratings
import LidarControl
import SensorSignal
import superRes


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
    # sys.exit() # commented for debug

# double check if display is available and acquire final infos
try:
    monitors = screeninfo.get_monitors()
    print("Total screen count:", len(monitors))
    for monitor in monitors:
        print(monitor)
    LCoS = monitors[0]
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
lidar_cfg.Nlight = int(128)
lidar_cfg.light_delay = 1.5
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
    win_modulation = tk.Tk()
    win_modulation.geometry(f"{LCoS.width}x{LCoS.height}+{LCoS.x}+{LCoS.y}")
    win_modulation.config(cursor="none")  # hide cursor
    win_modulation.overrideredirect(True)  # borderless
    win_modulation.attributes('-fullscreen', True)  # full screen
    panel = tk.Label(win_modulation)

    # light path modulation images
    modulation = np.zeros([5, LCoS.height, LCoS.width], dtype=np.uint8)
    modulation[0, :, :] = gratings.blazed_grating([LCoS.width, LCoS.height], pitch=[np.inf, np.inf])
    modulation[1, :, :] = gratings.blazed_grating([LCoS.width, LCoS.height], pitch=[100, 100])
    modulation[2, :, :] = gratings.blazed_grating([LCoS.width, LCoS.height], pitch=[100, -100])
    modulation[3, :, :] = gratings.blazed_grating([LCoS.width, LCoS.height], pitch=[-100, 100])
    modulation[4, :, :] = gratings.blazed_grating([LCoS.width, LCoS.height], pitch=[-100, -100])

    # main loop for LiDAR capturing
    zero_order = []
    sigs = []
    for multi_frame in range(5):

        # LCoS modulation
        shift_vec = np.array([(multi_frame - 1) % 2, (multi_frame - 1) // 2]) / 2
        img = ImageTk.PhotoImage(Image.fromarray(modulation[multi_frame, :, :]))
        panel.configure(image=img)
        panel.pack()
        win_modulation.update()
        time.sleep(0.1)

        # acquire physical sensor data
        sig = SensorSignal.acquire_signal(lidar, shift_vec=shift_vec, downsample_ratio=2)

        # progress info
        print(f" - Full frame {multi_frame} captured.")
        # log data
        if multi_frame > 0:
            sig -= zero_order * 0.1  # subtract the 0th order diffraction
            sigs.append(sig)
        else:
            zero_order = sig

        print()

    # end of for multi_frame in range(4)
    win_modulation.destroy()

    # IBP super-res
    sig_sr_lin = superRes.linear(*sigs)
    print(" - Linear Calculation done.")
    sig_sr_ibp = superRes.iterative(*sigs, iter_cnt=25, term_cond=5e-3)
    print(" - Iterative Calculation done.")

    # result display
    os.environ["DISPLAY"] = default_DISPLAY_env
    mpl.rcParams['image.interpolation'] = 'none'
    fig, axs = plt.subplots(nrows=2, ncols=3)
    # print raw distance
    im_dist = axs[0, 0].imshow(sigs[0].calc_dist(), cmap='viridis_r', vmin=0, vmax=15)
    fig.colorbar(im_dist, ax=axs[0, 0])
    axs[0, 0].set_title("Raw distance")
    # print raw intensity
    im_int = axs[1, 0].imshow(sigs[0].calc_intensity(), cmap='inferno', vmin=0, vmax=np.max(sigs[0].calc_intensity()))
    fig.colorbar(im_int, ax=axs[1, 0])
    axs[1, 0].set_title("Raw intensity")
    # print super-res distance (linear)
    im_dist = axs[0, 1].imshow(sig_sr_lin.calc_dist(), cmap='viridis_r', vmin=0, vmax=15)
    fig.colorbar(im_dist, ax=axs[0, 1])
    axs[0, 1].set_title("Linear super-res distance")
    # print super-res intensity (linear)
    im_int = axs[1, 1].imshow(sig_sr_lin.calc_intensity(), cmap='inferno', vmin=0, vmax=np.max(sig_sr_lin.calc_intensity()))
    fig.colorbar(im_int, ax=axs[1, 1])
    axs[1, 1].set_title("Linear super-res intensity")
    # print super-res distance (iterative)
    im_dist = axs[0, 2].imshow(sig_sr_ibp.calc_dist(), cmap='viridis_r', vmin=0, vmax=15)
    fig.colorbar(im_dist, ax=axs[0, 2])
    axs[0, 2].set_title("IBP super-res distance")
    # print super-res intensity (iterative) (it's pointless here since intensity is fully simulated)
    im_int = axs[1, 2].imshow(sig_sr_ibp.calc_intensity(), cmap='inferno', vmin=0, vmax=np.max(sig_sr_ibp.calc_intensity()))
    fig.colorbar(im_int, ax=axs[1, 2])
    axs[1, 2].set_title("IBP super-res intensity")
    print("display done.")
    plt.show()

# end of main program

except Exception as err:
    print("Error:", err)
    traceback.print_exc()

finally:
    # GC
    print("\n - Clean up.")
    os.environ["DISPLAY"] = default_DISPLAY_env
    sys.exit(0)
