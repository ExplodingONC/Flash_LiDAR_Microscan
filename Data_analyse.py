# import system modules
import os
import sys
import time
import datetime
# import utility modules
import numpy as np
import scipy.constants as const
import tkinter as tk
from tkinter import filedialog
from PIL import ImageTk, Image
import matplotlib.pyplot as plt
import matplotlib as mpl
import pickle
# import custom modules
import SensorSignal
import superRes

# timestamp
print("\n")
date = datetime.datetime.now().astimezone()
print(date.strftime("%Y-%m-%d %H:%M:%S.%f %Z %z"))
print()

# load data
file_dialog = tk.Tk()
file_dialog.withdraw()
FILEOPENOPTIONS = dict(title='Choose pickle data file',
                       defaultextension='.pkl',
                       filetypes=[('Pickle file','*.pkl'), ('All files','*.*')],
                       initialdir=os.getcwd())
file_path = filedialog.askopenfilename(**FILEOPENOPTIONS)
if file_path == "":
    print("No file is selected.")
    sys.exit()
if file_path.split(".")[-1] != "pkl":
    print("Incompatible file type.")
    sys.exit()
file_name = file_path.split("/")[-2] + "/" + file_path.split("/")[-1]
print(f" - Load \"{file_name}\" as data file.")
with open(file_path, 'rb') as data_file:
    zero_order = pickle.load(data_file)
    sigs = pickle.load(data_file)

# IBP super-res
sig_sr_lin = superRes.linear(*sigs)
print(" - Linear Calculation done.")
sig_sr_ibp, iter_cnt, final_err = superRes.iterative(*sigs, iter_cnt=25, term_cond=1e-4)
print(f" - Iterative Calculation done in {iter_cnt} iters. ", end="")
print(f"Remaining error is {100 * final_err:6.4f}%")

# result display
mpl.rcParams['image.interpolation'] = 'none'
fig, axs = plt.subplots(nrows=2, ncols=3)
# print raw distance
im_dist = axs[0, 0].imshow(sigs[0].calc_dist(), cmap='viridis_r', vmin=0, vmax=5)
fig.colorbar(im_dist, ax=axs[0, 0])
axs[0, 0].set_title("Raw distance")
# print raw intensity
im_int = axs[1, 0].imshow(sigs[0].calc_intensity(), cmap='inferno', vmin=0, vmax=np.max(sigs[0].calc_intensity()))
fig.colorbar(im_int, ax=axs[1, 0])
axs[1, 0].set_title("Raw intensity")
# print super-res distance (linear)
im_dist = axs[0, 1].imshow(sig_sr_lin.calc_dist(), cmap='viridis_r', vmin=0, vmax=5)
fig.colorbar(im_dist, ax=axs[0, 1])
axs[0, 1].set_title("Linear super-res distance")
# print super-res intensity (linear)
im_int = axs[1, 1].imshow(sig_sr_lin.calc_intensity(), cmap='inferno', vmin=0, vmax=np.max(sig_sr_lin.calc_intensity()))
fig.colorbar(im_int, ax=axs[1, 1])
axs[1, 1].set_title("Linear super-res intensity")
# print super-res distance (iterative)
im_dist = axs[0, 2].imshow(sig_sr_ibp.calc_dist(), cmap='viridis_r', vmin=0, vmax=5)
fig.colorbar(im_dist, ax=axs[0, 2])
axs[0, 2].set_title("IBP super-res distance")
# print super-res intensity (iterative) (it's pointless here since intensity is fully simulated)
im_int = axs[1, 2].imshow(sig_sr_ibp.calc_intensity(), cmap='inferno', vmin=0, vmax=np.max(sig_sr_ibp.calc_intensity()))
fig.colorbar(im_int, ax=axs[1, 2])
axs[1, 2].set_title("IBP super-res intensity")
print(" - Display done.")
plt.show()

print()