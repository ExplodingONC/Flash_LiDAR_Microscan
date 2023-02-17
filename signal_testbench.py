import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import SensorSignal
import superRes

fig, axs = plt.subplots(2, 2)

depth_map = np.ones([160, 200]) * 10
for width in range(1, 10):
    start = 20 * width - 5
    end = start + width
    depth_map[40:120, start:end] = 1
print("depth map done.")

sigs = []
for i in range(4):
    sigs.append(SensorSignal.SensorSignal([80, 100], downsample_ratio=2))
    sigs[-1].set_timing(8 / 60 * 1e-6)

sigs[0].sim_data(depth_map, downsample_ratio=2, shift_vec=[0, 0])
sigs[1].sim_data(depth_map, downsample_ratio=2, shift_vec=[0.5, 0])
sigs[2].sim_data(depth_map, downsample_ratio=2, shift_vec=[0, 0.5])
sigs[3].sim_data(depth_map, downsample_ratio=2, shift_vec=[0.5, 0.5])
print("simulation done.")

np.set_printoptions(formatter={'float': lambda x: "{0:5.2f}".format(x)})
sig_sr = superRes.iterative(*sigs, iter_cnt=25, term_cond=5e-3)
distance = sig_sr.calc_dist()
intensity = sig_sr.calc_intensity()
print("calculation done.")

mpl.rcParams['image.interpolation'] = 'none'
# print depth_map
im_depth = axs[0, 0].imshow(depth_map, cmap='viridis_r', vmin=0, vmax=15)
fig.colorbar(im_depth,ax=axs[0, 0])
# print raw data
im_raw = axs[1, 0].imshow(sigs[0].calc_dist(), cmap='viridis_r', vmin=0, vmax=15)
fig.colorbar(im_raw,ax=axs[1, 0])
# print super-res distance
im_dist = axs[0, 1].imshow(distance, cmap='viridis_r', vmin=0, vmax=15)
fig.colorbar(im_dist,ax=axs[0, 1])
# print super-res intensity
im_inten = axs[1, 1].imshow(intensity, cmap='inferno', vmin=0, vmax=np.max(intensity))
fig.colorbar(im_inten,ax=axs[1, 1])
print("display done.")
plt.show()
