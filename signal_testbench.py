import numpy as np
import SensorSignal
import superRes

depth_map = (np.arange(16 * 24) / 4 + 1).reshape((16, 24))
print(depth_map)

sig = []
for i in range(4):
    sig.append(SensorSignal.SensorSignal([8, 12], downsample_ratio=2))
    sig[-1].T_0 = 8 / 60 * 1e-6

sig[0].sim_data(depth_map, downsample_ratio=2, shift_vec=[0, 0])
sig[1].sim_data(depth_map, downsample_ratio=2, shift_vec=[0.5, 0])
sig[2].sim_data(depth_map, downsample_ratio=2, shift_vec=[0, 0.5])
sig[3].sim_data(depth_map, downsample_ratio=2, shift_vec=[0.5, 0.5])

sig_sr = superRes.linear(sig[0],sig[1],sig[2],sig[3])

distance = sig_sr.calc_dist()
print(distance)
