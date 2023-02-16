import numpy as np
import SensorSignal

sig = SensorSignal.SensorSignal([8, 12])
sig.T_0 = 8 / 60 * 1e-6

depth_map = (np.arange(16*24)/10+1).reshape((16,24))
# depth_map = np.ones((16,24))*7
print(depth_map)

sig.sim_data(depth_map, downsample_ratio=2, shift_vec=[0, 0])
distance = sig.calc_dist()
print(distance)