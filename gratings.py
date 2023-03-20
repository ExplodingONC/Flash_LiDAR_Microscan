import numpy as np


def binary_grating(res=[1920, 1080], pitch=[50, 50], range=[0, 255], dtype=np.uint8):
    x = np.linspace(0, res[0] - 1, num=res[0])
    y = np.linspace(0, res[1] - 1, num=res[1])
    X, Y = np.meshgrid(x, y, indexing='xy')
    mask = ((X % pitch[0]) / pitch[0] + (Y % pitch[1]) / pitch[1]) % 1 >= 0.5
    grating = np.array(range[0] + mask * (range[1] - range[0]), dtype=dtype)
    return grating


def blazed_grating(res=[1920, 1080], pitch=[50, 50], range=[0, 255], dtype=np.uint8):
    x = np.linspace(0, res[0] - 1, num=res[0])
    y = np.linspace(0, res[1] - 1, num=res[1])
    X, Y = np.meshgrid(x, y, indexing='xy')
    mask = ((X % pitch[0]) / pitch[0] + (Y % pitch[1]) / pitch[1]) % 1
    grating = np.array(range[0] + mask * (range[1] - range[0]), dtype=dtype)
    return grating


if __name__ == "__main__":
    binary = binary_grating(res=[12, 10], pitch=[6, 5])
    print(binary)
    blazed = blazed_grating(res=[12, 10], pitch=[6, 5])
    print(blazed)
