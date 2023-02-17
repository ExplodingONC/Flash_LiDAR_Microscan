import numpy as np
import scipy.constants as const


def rebin(a, shape):
    M, N = np.shape(a)
    m, n = shape
    assert M % m == 0 or m % M == 0
    assert N % n == 0 or n % N == 0
    if m < M:
        return a.reshape((m, M // m, n, N // n)).mean(3).mean(1)
    else:
        return np.repeat(np.repeat(a, m // M, axis=0), n // N, axis=1)


def translate(a, vector, mode='edge'):
    vector = np.array(vector, dtype=int)
    x, y = vector
    if x > 0:
        if y > 0:
            return np.pad(a, ((y, 0), (x, 0)), mode=mode)[:-y, :-x]
        else:
            return np.pad(a, ((0, -y), (x, 0)), mode=mode)[-y:, :-x]
    else:
        if y > 0:
            return np.pad(a, ((y, 0), (0, -x)), mode=mode)[:-y, -x:]
        else:
            return np.pad(a, ((0, -y), (0, -x)), mode=mode)[-y:, -x:]


class SensorSignal:

    # physical
    resolution: np.ndarray = [72, 96]
    downsample_ratio: int = 1
    shift_vector: np.ndarray = [0, 0]
    T_0: float = 133e-9
    # data
    data: np.ndarray = []
    delta_F1: np.ndarray = []
    delta_F2: np.ndarray = []

    # init
    def __init__(self, resolution, downsample_ratio=1):
        self.resolution = resolution
        self.downsample_ratio = downsample_ratio
        self.data = np.zeros([4, 2] + list(resolution))

    # type fix
    def __setattr__(self, name, value):
        if name == 'resolution' and not isinstance(value, np.ndarray):
            value = np.array(value)
        if name == 'shift_vector' and not isinstance(value, np.ndarray):
            value = np.array(value)
        super().__setattr__(name, value)

    # input data
    def use_data(self, raw_data):
        self.data = raw_data

    def set_timing(self, T_0):
        self.T_0 = T_0

    # simulate data
    def sim_data(self, depth_map, downsample_ratio=2, shift_vec=[0, 0]):
        assert tuple(self.resolution * downsample_ratio) == np.shape(depth_map)
        self.shift_vector = shift_vec
        self.downsample_ratio = downsample_ratio
        map_shift = np.rint(self.shift_vector * downsample_ratio)  # still float here
        depth_map = translate(depth_map, -map_shift)
        raw_data = np.zeros((4, 2) + np.shape(depth_map))
        # signal return time
        np.seterr(invalid='ignore')
        return_time = np.zeros((2,) + np.shape(depth_map))
        return_time[0, :, :] = 2 * depth_map / const.speed_of_light + self.T_0 / 2
        return_time[1, :, :] = 2 * depth_map / const.speed_of_light
        # F1, F2
        for subframe in range(2):
            raw_data[subframe, 0, :, :] \
                = (return_time[subframe, :, :] < self.T_0) * (self.T_0 - return_time[subframe, :, :]) \
                + ((self.T_0 <= return_time[subframe, :, :]) & (return_time[subframe, :, :] < 2 * self.T_0)) * (return_time[subframe, :, :] - self.T_0) \
                + ((2 * self.T_0 <= return_time[subframe, :, :]) & (return_time[subframe, :, :] < 3 * self.T_0)) * (3 * self.T_0 - return_time[subframe, :, :]) \
                + (3 * self.T_0 <= return_time[subframe, :, :]) * np.zeros(np.shape(return_time[subframe, :, :]))
            raw_data[subframe, 0, :, :] = raw_data[subframe, 0, :, :] * (depth_map ** float(-2))
            raw_data[subframe, 1, :, :] \
                = (return_time[subframe, :, :] < self.T_0) * (return_time[subframe, :, :] + self.T_0 - self.T_0) \
                + ((self.T_0 <= return_time[subframe, :, :]) & (return_time[subframe, :, :] < 2 * self.T_0)) * (2 * self.T_0 - return_time[subframe, :, :]) \
                + ((2 * self.T_0 <= return_time[subframe, :, :]) & (return_time[subframe, :, :] < 3 * self.T_0)) * (return_time[subframe, :, :] - 2 * self.T_0) \
                + ((3 * self.T_0 <= return_time[subframe, :, :]) & (return_time[subframe, :, :] < 4 * self.T_0)) * (4 * self.T_0 - return_time[subframe, :, :]) \
                + (4 * self.T_0 <= return_time[subframe, :, :]) * np.zeros(np.shape(return_time[subframe, :, :]))
            raw_data[subframe, 1, :, :] = raw_data[subframe, 1, :, :] * (depth_map ** float(-2))
        # F3, F4
        raw_data[2, 0, :, :] = raw_data[0, 1, :, :]
        raw_data[2, 1, :, :] = raw_data[0, 0, :, :]
        raw_data[3, 0, :, :] = raw_data[1, 1, :, :]
        raw_data[3, 1, :, :] = raw_data[1, 0, :, :]
        # remove invalid data
        raw_data = np.nan_to_num(raw_data, copy=False, nan=0, posinf=0, neginf=0)
        # down sample
        for subframe in range(4):
            self.data[subframe, 0, :, :] = rebin(raw_data[subframe, 0, :, :], self.resolution)
            self.data[subframe, 1, :, :] = rebin(raw_data[subframe, 1, :, :], self.resolution)

    # calculate intensity
    def calc_intensity(self):
        return np.sum(self.data, axis=(0, 1)) / 8

    # calculate deltas
    def calc_delta(self):
        # delta_F1 = (F1_Ch2 - F1_Ch1) + (F3_Ch1 - F3_Ch2)
        self.delta_F1 = (self.data[0, 1, :, :] - self.data[0, 0, :, :]) \
            + (self.data[2, 0, :, :] - self.data[2, 1, :, :])
        # delta_F2 = (F2_Ch2 - F2_Ch1) + (F4_Ch1 - F4_Ch2)
        self.delta_F2 = (self.data[1, 1, :, :] - self.data[1, 0, :, :]) \
            + (self.data[3, 0, :, :] - self.data[3, 1, :, :])

    # extract distance
    def calc_dist(self):
        self.calc_delta()
        # mask for valid data (non-zero)
        if np.any(np.abs(self.delta_F1) > 1) or np.any(np.abs(self.delta_F2) > 1):
            # for actual 12-bit uint data (which has noise)
            valid_mask = np.logical_or(
                (np.abs(self.delta_F1) >= 10),
                (np.abs(self.delta_F2) >= 10)
            )
        else:
            # for simulation data
            valid_mask = np.logical_or(
                (np.abs(self.delta_F1) > 0),
                (np.abs(self.delta_F2) > 0)
            )
        np.seterr(invalid='ignore')
        depth_map \
            = (self.delta_F1 > 0) \
            * (self.delta_F2 / (abs(self.delta_F1) + abs(self.delta_F2)) + 1) \
            / 4 * const.speed_of_light * self.T_0 \
            + (self.delta_F1 <= 0) \
            * (-self.delta_F2 / (abs(self.delta_F1) + abs(self.delta_F2)) + 3) \
            / 4 * const.speed_of_light * self.T_0
        depth_map = valid_mask * depth_map
        depth_map = np.nan_to_num(depth_map, copy=False, nan=np.inf, posinf=np.inf, neginf=np.inf)
        return depth_map
