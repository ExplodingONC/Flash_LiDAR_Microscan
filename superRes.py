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


def linear_range(*sigs: SensorSignal):
    # basic info
    sr_res = sigs[0].resolution * 2
    sr_downsample_ratio = sigs[0].downsample_ratio / 2
    sr_sig = SensorSignal.SensorSignal(sr_res, sr_downsample_ratio)
    sr_sig.shift_vector = sigs[0].shift_vector
    sr_sig.set_timing(sigs[0].T_0)
    # linear super res
    depth_map = np.zeros(sr_res)
    for k, signal in enumerate(sigs):
        slice = signal.calc_dist()
        slice = SensorSignal.rebin(slice, sr_res)
        slice = SensorSignal.translate(slice, 2 * signal.shift_vector)
        depth_map += slice
    depth_map = depth_map / (k + 1)
    reflect_map = np.zeros(sr_res)
    for k, signal in enumerate(sigs):
        slice = signal.calc_reflect()
        slice = SensorSignal.rebin(slice, sr_res)
        slice = SensorSignal.translate(slice, 2 * signal.shift_vector)
        reflect_map += slice
    reflect_map = reflect_map / (k + 1)
    sr_sig.sim_data(depth_map, reflect_map=reflect_map, downsample_ratio=1, shift_vec=[0, 0])
    return sr_sig


def back_projection(sig_ref: SensorSignal, sig_iter: SensorSignal):
    dist_ref = sig_ref.calc_dist()
    dist_iter = sig_iter.calc_dist()
    reflect_ref = sig_ref.calc_reflect()
    reflect_iter = sig_iter.calc_reflect()
    # valid if both returns a valid distance
    valid_mask = np.logical_and(np.isfinite(dist_ref), np.isfinite(dist_iter))
    # need compensation if reference is out of range but iteration still valid
    compensate_mask = np.logical_and(~np.isfinite(dist_ref), np.isfinite(dist_iter))
    # generate difference for iteration
    dist_diff = dist_ref - dist_iter
    dist_diff[~valid_mask] = 0
    dist_diff[compensate_mask] = np.inf
    reflect_diff = reflect_ref - reflect_iter
    reflect_diff[~valid_mask] = 0
    reflect_diff[compensate_mask] = 0
    return dist_diff, reflect_diff


def iterative(*sigs_ref: SensorSignal, iter_cnt=25, term_cond=0):
    # prepare iteration start point
    sig_cnt = len(sigs_ref)
    sig_linear = linear(*sigs_ref)
    depth_iter = sig_linear.calc_dist()
    reflect_iter = sig_linear.calc_reflect()
    # prepare iteration vals (must use deep copy for objects)
    sigs_iter = copy.deepcopy(sigs_ref)
    err_mean = 0
    depth_mean = 1
    # iteration
    for iter in range(iter_cnt):
        # prepare error log
        err_dist_total = np.zeros(depth_iter.shape)
        err_reflect_total = np.zeros(depth_iter.shape)
        for sig_ref, sig_iter in zip(sigs_ref, sigs_iter):
            # simulate projected signal
            sig_iter.sim_data(depth_iter, reflect_map=reflect_iter, downsample_ratio=2,
                              shift_vec=sig_iter.shift_vector - sigs_iter[0].shift_vector)
            # back projection
            err_dist, err_reflect = back_projection(sig_ref, sig_iter)
            err_dist = SensorSignal.rebin(err_dist, sig_iter.resolution * 2)
            err_dist = SensorSignal.translate(err_dist, (sig_iter.shift_vector - sigs_iter[0].shift_vector) * 2)
            err_reflect = SensorSignal.rebin(err_reflect, sig_iter.resolution * 2)
            err_reflect = SensorSignal.translate(err_reflect, (sig_iter.shift_vector - sigs_iter[0].shift_vector) * 2)
            # update error log
            err_dist_total = err_dist_total + err_dist
            err_reflect_total = err_reflect_total + err_reflect
        # update iterative depth map
        err_dist_total = err_dist_total / sig_cnt
        err_reflect_total = err_reflect_total / sig_cnt
        depth_iter = depth_iter + err_dist_total * 0.5
        reflect_iter = reflect_iter + err_reflect_total * 0.5
        # check terminate condition
        if term_cond > 0 and np.all(np.isfinite(err_dist_total)):
            valid_mask = np.logical_and(np.isfinite(err_dist_total), np.isfinite(depth_iter))
            if not np.any(valid_mask):
                break
            err_mean = np.mean(np.abs(err_dist_total), where=valid_mask)  # (numpy 1.20 needed)
            depth_mean = np.mean(depth_iter, where=valid_mask)  # (numpy 1.20 needed)
            # print(f"{iter+1}: Err={err_mean:.2e} Avg={depth_mean:.2e} Acc={err_mean/depth_mean:.5f}")
            if err_mean < depth_mean * term_cond:
                break
    # generate result
    sig_ret = SensorSignal.SensorSignal(depth_iter.shape)
    sig_ret.sim_data(depth_iter, reflect_map=reflect_iter, downsample_ratio=1, shift_vec=[0, 0])
    return sig_ret, iter + 1, err_mean / depth_mean
