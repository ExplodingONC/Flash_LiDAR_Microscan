# import system modules
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes, mark_inset
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
                       filetypes=[('Pickle file', '*.pkl'), ('All files', '*.*')],
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
    assert isinstance(zero_order, SensorSignal.SensorSignal)
    assert isinstance(sigs, list)
    for i in range(len(sigs)):
        assert isinstance(sigs[i], SensorSignal.SensorSignal)

# IBP super-res
sig_sr_lin = superRes.linear_range(*sigs)
print(" - Linear Calculation done.")
sig_sr_ibp, iter_cnt, final_err = superRes.iterative(*sigs, iter_cnt=10, term_cond=1e-4)
print(f" - Iterative Calculation done in {iter_cnt} iters. ", end="")
print(f"Remaining error is {100 * final_err:6.4f}%")

# result display
mpl.rcParams['image.interpolation'] = 'none'
fig_1, axs_1 = plt.subplots(nrows=1, ncols=1, figsize=(6.4, 4))
fig_2, axs_2 = plt.subplots(nrows=1, ncols=1, figsize=(6.4, 4))
range_min = np.min(zero_order.calc_dist()) * 0.95
range_max = np.max(zero_order.calc_dist()) * 1.05
# select data
sig_used = sig_sr_ibp   # zero_order, sig_sr_lin, sig_sr_ibp, sigs[0..3]
location = 'lower left'
zoom_x_lim = (sig_used.resolution[1] * 0.4, sig_used.resolution[1] * 0.6)
zoom_y_lim = (sig_used.resolution[0] * 0.6, sig_used.resolution[0] * 0.4)
# print intensity
im_1 = axs_1.imshow(sig_used.calc_intensity(), cmap='Greys_r', vmin=0, vmax=np.max(sig_used.calc_intensity()))
axs_1_insert = zoomed_inset_axes(axs_1, 2, loc=location)
axs_1_insert.imshow(sig_used.calc_intensity(), cmap='Greys_r', vmin=0, vmax=np.max(sig_used.calc_intensity()))
axs_1_insert.set_xlim(zoom_x_lim)
axs_1_insert.set_ylim(zoom_y_lim)
axs_1_insert.set_xticks([])
axs_1_insert.set_yticks([])
axs_1.indicate_inset_zoom(axs_1_insert, edgecolor="red")
fig_1.colorbar(im_1, ax=axs_1)
axs_1.set_title("Intensity")
# print distance
im_2 = axs_2.imshow(sig_used.calc_dist(), cmap='inferno_r', vmin=range_min, vmax=range_max)
axs_2_insert = zoomed_inset_axes(axs_2, 2, loc=location)
axs_2_insert.imshow(sig_used.calc_dist(), cmap='inferno_r', vmin=range_min, vmax=range_max)
axs_2_insert.set_xlim(zoom_x_lim)
axs_2_insert.set_ylim(zoom_y_lim)
axs_2_insert.set_xticks([])
axs_2_insert.set_yticks([])
axs_2.indicate_inset_zoom(axs_2_insert, edgecolor="red")
fig_2.colorbar(im_2, ax=axs_2)
axs_2.set_title("Distance (m)")
print(" - Display done.")
plt.show()

print()
