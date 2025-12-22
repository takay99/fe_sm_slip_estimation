from math import exp


def heuristic_schedule(
    input: float, input_dot: float, sigma: float, singma_dot: float
) -> float:
    return exp(
        (-1 / 2) * (((input**2) / (sigma**2)) + ((input_dot**2) / (singma_dot**2)))
    )



