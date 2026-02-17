import numpy as np
import pandas as pd
from scipy.integrate import odeint
from vehicle_model import rk4_step
import matplotlib.pyplot as plt
from typing import Annotated


def sim_vehicle_model_velocity(delta: float, tau: Vector4, velocity_x,velocity_y, time,dot_phai):
   
    m = 1500
    I = 1500
    lf = 1.51
    lr = 1.49
    a = 1.5
    b = 1.5
    Kf = 10**5
    Kr = 10**5
    radius = 0.3

    # Ft = 



    return 0


if __name__ == "__main__":


    # A. 二重リストで直接定義 (3行1列)
    x = np.array([[1.0], 
                [2.0], 
                [3.0]])

    # B. 1次元配列で作ってから変形 (推奨: 書きやすい)
    # shape (3,) -> (3, 1)
    v = np.array([1.0, 2.0, 3.0]).reshape(-1, 1)

    print(f"Shape: {v.shape}") # (3, 1)