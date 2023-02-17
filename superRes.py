import copy
import numpy as np
import SensorSignal


def linear(*sigs: SensorSignal):
    # basic info
    sr_res = sigs[0].resolution * 2
    sr_downsample_ratio = sigs[0].downsample_ratio / 2
    sr_sig = SensorSignal.SensorSignal(sr_res, sr_downsample_ratio)
    sr_sig.shift_vector = sigs[0].shift_vector
    sr_sig.set_timing(sigs[0].T_0)
    # linear super res
    for subframe in range(4):
        for channel in range(2):
            for k, signal in enumerate(sigs):
                slice = signal.data[subframe, channel, :, :]
                slice = SensorSignal.rebin(slice, sr_res)
                slice = SensorSignal.translate(slice, 2 * signal.shift_vector)
                sr_sig.data[subframe, channel, :, :] += slice
            sr_sig.data[subframe, channel, :, :] = sr_sig.data[subframe, channel, :, :] / (k + 1)
    return sr_sig


def back_projection(sig_ref: SensorSignal, sig_iter: SensorSignal):
    dist_ref = sig_ref.calc_dist()
    dist_iter = sig_iter.calc_dist()
    # valid if both returns a valid distance
    valid_mask = np.logical_and(np.isfinite(dist_ref), np.isfinite(dist_iter))
    # need compensation if reference is out of range but iteration still valid
    compensate_mask = np.logical_and(~np.isfinite(dist_ref), np.isfinite(dist_iter))
    # generate difference for iteration
    dist_diff = dist_ref - dist_iter
    dist_diff[~valid_mask] = 0
    dist_diff[compensate_mask] = np.inf
    return dist_diff


def iterative(*sigs_ref: SensorSignal, iter_cnt=25):
    # prepare iteration start point
    sig_cnt = len(sigs_ref)
    depth_iter = linear(*sigs_ref).calc_dist()
    # prepare iteration vals (must use deep copy for objects)
    sigs_iter = copy.deepcopy(sigs_ref)
    # iteration
    for iter in range(iter_cnt):
        # prepare error log
        err_total = np.zeros(depth_iter.shape)
        for sig_ref, sig_iter in zip(sigs_ref, sigs_iter):
            # simulate projected signal
            sig_iter.sim_data(depth_iter, downsample_ratio=2,
                              shift_vec=sig_iter.shift_vector - sigs_iter[0].shift_vector)
            # back projection
            err_iter = back_projection(sig_ref, sig_iter)
            err_iter = SensorSignal.rebin(err_iter, sig_iter.resolution * 2)
            err_iter = SensorSignal.translate(err_iter, sig_iter.shift_vector * 2)
            # update error log
            err_total = err_total + err_iter
        # update iterative depth map
        err_total = err_total / sig_cnt
        depth_iter = depth_iter + err_total * 0.5
    # generate result
    sig_ret = SensorSignal.SensorSignal(depth_iter.shape)
    sig_ret.sim_data(depth_iter, downsample_ratio=1, shift_vec=[0, 0])
    return sig_ret
