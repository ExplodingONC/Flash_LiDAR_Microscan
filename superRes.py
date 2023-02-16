import numpy as np
import SensorSignal


def linear(*sig: SensorSignal):
    # basic info
    sr_res = sig[0].resolution * 2
    sr_downsample_ratio = sig[0].downsample_ratio / 2
    sr_sig = SensorSignal.SensorSignal(sr_res, sr_downsample_ratio)
    sr_sig.shift_vector = sig[0].shift_vector
    sr_sig.set_timing(sig[0].T_0)
    # linear super res
    for subframe in range(4):
        for channel in range(2):
            for frame in range(4):
                slice = sig[frame].data[subframe, channel, :, :]
                slice = SensorSignal.rebin(slice, sr_res)
                slice = SensorSignal.translate(slice, 2 * sig[frame].shift_vector)
                sr_sig.data[subframe, channel, :, :] += slice
            sr_sig.data[subframe, channel, :, :] = sr_sig.data[subframe, channel, :, :] / 4
    return sr_sig


def iterative():
    pass # WIP
