import numpy as np
import pandas as pd

def runge_kutta(f, y, t,t_step):

    k1 = f(t, y)
    k2 = f(t + t_step / 2, y + k1 * t_step / 2)
    k3 = f(t + t_step / 2, y + k2 * t_step / 2)
    k4 = f(t + t_step, y + k3 * t_step)
    y_next = y + (k1 + 2 * k2 + 2 * k3 + k4) * t_step / 6
    return y_next
    

    