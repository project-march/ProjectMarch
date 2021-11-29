import numpy as np


def cos(angle_deg):
    return np.cos(np.deg2rad(angle_deg))


def sin(angle_deg):
    return np.sin(np.deg2rad(angle_deg))


def tan(angle_deg):
    return np.tan(np.deg2rad(angle_deg))


def acos(a):
    return np.rad2deg(np.arccos(a))


def asin(a):
    return np.rad2deg(np.arcsin(a))


def atan(a):
    return np.rad2deg(np.arctan(a))
