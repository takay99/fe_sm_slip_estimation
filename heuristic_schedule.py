from math import exp


def heuristic_schedule(
    input: float, input_dot: float, sigma: float, singma_dot: float
) -> float:
    return exp(
        (-1 / 2) * (((input**2) / (sigma**2)) + ((input_dot**2) / (singma_dot**2)))
    )


if __name__ == "__main__":
    # Test Case 1: All positive inputs
    result1 = heuristic_schedule(1.0, 1.0, 0.18, 0.18)
    print(f"Test Case 1 (all positive): {result1}")

    # Test Case 2: Mixed positive and negative inputs
    result2 = heuristic_schedule(-1.0, 1.0, 2.0, 2.0)
    print(f"Test Case 2 (mixed signs): {result2}")

    # Test Case 3: Zero inputs
    result3 = heuristic_schedule(0.0, 0.0, 1.0, 1.0)
    print(f"Test Case 3 (zero inputs): {result3}")

    # Test Case 4: Different sigma values
    result4 = heuristic_schedule(1.0, 1.0, 1.0, 3.0)
    print(f"Test Case 4 (different sigma): {result4}")
