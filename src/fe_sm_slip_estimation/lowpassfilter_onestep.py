import numpy as np

class LowPassFilterOnestep:
    def __init__ (self, cutoff_freq: float, T: float):
        self.cutoff_freq = cutoff_freq
        self.T = T
        self.prev_y = 0.0
        self.is_first_run = True
        self.omega_c_T = cutoff_freq * 2 * np.pi * T
        self.alpha = self.omega_c_T / (1 + self.omega_c_T)


    def filter(self, y: float) -> float:
        if self.is_first_run:
            self.prev_y = y
            self.is_first_run = False
        filtered_y = self.prev_y + self.alpha * (y - self.prev_y)
        self.prev_y = filtered_y
        return filtered_y