import numpy as np
import SensorSignal
import superRes

# depth_map = (np.arange(12 * 16) / 10 + 1).reshape((12, 16))
depth_strip = np.block([np.ones([12,1]),np.ones([12,7])*10])
depth_map = np.block([depth_strip,depth_strip])

print(depth_map)

sigs = []
for i in range(4):
    sigs.append(SensorSignal.SensorSignal([6, 8], downsample_ratio=2))
    sigs[-1].set_timing(8 / 60 * 1e-6)

sigs[0].sim_data(depth_map, downsample_ratio=2, shift_vec=[0, 0])
sigs[1].sim_data(depth_map, downsample_ratio=2, shift_vec=[0.5, 0])
sigs[2].sim_data(depth_map, downsample_ratio=2, shift_vec=[0, 0.5])
sigs[3].sim_data(depth_map, downsample_ratio=2, shift_vec=[0.5, 0.5])

np.set_printoptions(formatter={'float': lambda x: "{0:5.2f}".format(x)})
sig_sr = superRes.iterative(*sigs)

distance = sig_sr.calc_dist()
print(distance)
