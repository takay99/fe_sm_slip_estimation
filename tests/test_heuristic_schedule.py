from fe_sm_slip_estimation.heuristic_schedule import heuristic_schedule
import pytest
from math import isclose


def test_heuristic_schedule_positive_inputs():
    # Test Case 1: All positive inputs
    expected = 0.7788007830714049
    assert isclose(heuristic_schedule(1.0, 1.0, 2.0, 2.0), expected)


def test_heuristic_schedule_mixed_signs():
    # Test Case 2: Mixed positive and negative inputs
    expected = 0.7788007830714049
    assert isclose(heuristic_schedule(-1.0, 1.0, 2.0, 2.0), expected)


def test_heuristic_schedule_zero_inputs():
    # Test Case 3: Zero inputs
    expected = 1.0
    assert isclose(heuristic_schedule(0.0, 0.0, 1.0, 1.0), expected)


def test_heuristic_schedule_different_sigma():
    # Test Case 4: Different sigma values
    expected = 0.5737534207374327
    assert isclose(heuristic_schedule(1.0, 1.0, 1.0, 3.0), expected)


def test_heuristic_schedule_large_inputs():
    # Test Case 5: Large inputs
    expected = 0.0  # Should be very close to 0
    assert isclose(heuristic_schedule(100.0, 100.0, 1.0, 1.0), expected, abs_tol=1e-9)
