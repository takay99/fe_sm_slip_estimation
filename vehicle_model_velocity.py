import numpy as np
import pandas as pd
from scipy.integrate import odeint
from vehicle_model import rk4_step
import matplotlib.pyplot as plt
from typing import Annotated
import math

Vector4 = Annotated[np.ndarray, "4次元ベクトル"]


def sim_vehicle_model_velocity(delta: float, tau: Vector4, velocity_x,velocity_y, time ,dot_phai ):
   
    m = 1500
    I = 1500
    center2rightwheel = 0.7
    center2leftwheel = 0.7
    wheelbase_f = 1.5
    wheelbase_r = 1.8
    Kf = 10**5
    Kr = 10**5
    radius = 0.3

    slip_angle_fl = delta - (velocity_y +wheelbase_f * dot_phai) / (velocity_x - center2leftwheel * dot_phai)
    slip_angle_fr = delta - (velocity_y +wheelbase_f * dot_phai) / (velocity_x + center2rightwheel * dot_phai)
    slip_angle_rl = -(velocity_y - wheelbase_r * dot_phai) / (velocity_x - center2leftwheel * dot_phai)
    slip_angle_rr = -(velocity_y - wheelbase_r * dot_phai) / (velocity_x + center2rightwheel * dot_phai)

    Fs_fl = -Kf * slip_angle_fl
    Fs_fr = -Kf * slip_angle_fr
    Fs_rl = -Kr * slip_angle_rl
    Fs_rr = -Kr * slip_angle_rr
    Fs_vector = np.array([Fs_fl, Fs_fr, Fs_rl, Fs_rr]).reshape(-1, 1)

    Ft_fl = tau[0] * radius
    Ft_fr = tau[1] * radius
    Ft_rl = tau[2] * radius
    Ft_rr = tau[3] * radius
    Ft_vector = np.array([Ft_fl, Ft_fr, Ft_rl, Ft_rr]).reshape(-1, 1)
    
    Fx = np.diag([math.cos(delta), math.cos(delta), 1, 1]) @ (Fs_vector + Ft_vector)

    return 0


if __name__ == "__main__":


    # A. 二重リストで直接定義 (3行1列)
    x = np.array([[1.0], 
                [2.0], 
                [3.0]])

    # B. 1次元配列で作ってから変形 (推奨: 書きやすい)
    # shape (3,) -> (3, 1)
    v = np.array([1.0, 2.0, 3.0]).reshape(-1, 1)

    print('\f',v) # (3, 1)
    print('\f',x) # (3, 1)
    print(x.shape)
    print(v.shape)