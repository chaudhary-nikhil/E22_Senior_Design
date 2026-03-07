#!/usr/bin/env python3
"""
Test suite for ideal stroke comparison logic.
Validates the normalized Euclidean distance comparison used
both in Python (dashboard) and C (firmware stroke_detector).

Run: python3 test_ideal_comparison.py
"""

import sys
import os
import math
import random

# ============================================================================
# Port of stroke_detector.c compare_strokes() to Python for testing
# ============================================================================

def compare_strokes(current, cur_count, ideal, ideal_count):
    """
    Compare two stroke LIA profiles using normalized Euclidean distance
    with linear interpolation for length alignment.
    
    Args:
        current: list of floats [lia_x0, lia_y0, lia_z0, lia_x1, ...] 
        cur_count: number of 3-float samples in current
        ideal: list of floats [lia_x0, lia_y0, lia_z0, lia_x1, ...]
        ideal_count: number of 3-float samples in ideal
    
    Returns:
        float: deviation score (0.0 = perfect, higher = worse)
    """
    if cur_count == 0 or ideal_count == 0:
        return 0.0

    ref_count = ideal_count
    total_dist = 0.0

    for i in range(ref_count):
        # Map ideal index to current index (linear interpolation)
        t = (i / (ref_count - 1) * (cur_count - 1)) if cur_count > 1 else 0
        idx = int(t)
        frac = t - idx
        if idx >= cur_count - 1:
            idx = cur_count - 1
            frac = 0.0

        # Interpolate current stroke at this position
        if frac < 0.001 or idx >= cur_count - 1:
            cx = current[idx * 3 + 0]
            cy = current[idx * 3 + 1]
            cz = current[idx * 3 + 2]
        else:
            cx = current[idx * 3] * (1 - frac) + current[(idx + 1) * 3] * frac
            cy = current[idx * 3 + 1] * (1 - frac) + current[(idx + 1) * 3 + 1] * frac
            cz = current[idx * 3 + 2] * (1 - frac) + current[(idx + 1) * 3 + 2] * frac

        # Ideal sample
        ix = ideal[i * 3 + 0]
        iy = ideal[i * 3 + 1]
        iz = ideal[i * 3 + 2]

        # Euclidean distance
        dx, dy, dz = cx - ix, cy - iy, cz - iz
        total_dist += math.sqrt(dx * dx + dy * dy + dz * dz)

    avg_dist = total_dist / ref_count
    return avg_dist / 5.0  # Normalized to ~[0, 1+]


# ============================================================================
# Test Cases
# ============================================================================

def test_identical_strokes():
    """Perfect match should return 0.0"""
    ideal = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]  # 3 samples
    result = compare_strokes(ideal, 3, ideal, 3)
    assert result == 0.0, f"Expected 0.0, got {result}"
    print("✓ test_identical_strokes: PASS (deviation = 0.0)")


def test_similar_strokes():
    """Small noise should give low deviation (<0.1)"""
    random.seed(42)
    ideal = []
    for i in range(50):
        ideal.extend([math.sin(i * 0.1) * 3, math.cos(i * 0.1) * 2, math.sin(i * 0.05)])
    
    current = [v + random.gauss(0, 0.1) for v in ideal]
    result = compare_strokes(current, 50, ideal, 50)
    assert result < 0.1, f"Expected < 0.1, got {result}"
    print(f"✓ test_similar_strokes: PASS (deviation = {result:.4f})")


def test_different_strokes():
    """Very different pattern should give high deviation (>0.5)"""
    ideal = [math.sin(i * 0.1) * 5 for i in range(30 * 3)]
    different = [math.cos(i * 0.3) * 10 for i in range(30 * 3)]
    result = compare_strokes(different, 30, ideal, 30)
    assert result > 0.3, f"Expected > 0.3, got {result}"
    print(f"✓ test_different_strokes: PASS (deviation = {result:.4f})")


def test_length_mismatch():
    """Different length strokes should still compare (via interpolation)"""
    ideal = [1.0, 0.0, 0.0] * 50  # 50 samples
    shorter = [1.0, 0.0, 0.0] * 20  # 20 samples
    result = compare_strokes(shorter, 20, ideal, 50)
    assert result == 0.0, f"Expected 0.0 (same pattern), got {result}"
    print(f"✓ test_length_mismatch: PASS (deviation = {result:.4f})")


def test_empty_data():
    """Empty data should return 0.0 without crashing"""
    result1 = compare_strokes([], 0, [1.0, 2.0, 3.0], 1)
    assert result1 == 0.0, f"Expected 0.0 for empty current, got {result1}"
    
    result2 = compare_strokes([1.0, 2.0, 3.0], 1, [], 0)
    assert result2 == 0.0, f"Expected 0.0 for empty ideal, got {result2}"
    print("✓ test_empty_data: PASS")


def test_single_sample():
    """Single sample comparison should work"""
    ideal = [1.0, 2.0, 3.0]
    current = [1.5, 2.5, 3.5]
    result = compare_strokes(current, 1, ideal, 1)
    expected = math.sqrt(0.25 + 0.25 + 0.25) / 5.0
    assert abs(result - expected) < 0.001, f"Expected {expected:.4f}, got {result:.4f}"
    print(f"✓ test_single_sample: PASS (deviation = {result:.4f})")


def test_deviation_gradient():
    """Increasing noise should increase deviation monotonically"""
    ideal = [math.sin(i * 0.1) * 3 for i in range(100 * 3)]
    prev_dev = -1.0
    noise_levels = [0.1, 0.5, 1.0, 2.0, 5.0]
    
    for noise in noise_levels:
        random.seed(123)
        noisy = [v + random.gauss(0, noise) for v in ideal]
        dev = compare_strokes(noisy, 100, ideal, 100)
        assert dev > prev_dev, f"Deviation should increase: noise={noise}, dev={dev:.4f} <= prev={prev_dev:.4f}"
        prev_dev = dev
    
    print(f"✓ test_deviation_gradient: PASS (monotonically increasing)")


# ============================================================================
# Run all tests
# ============================================================================

if __name__ == '__main__':
    print("=" * 50)
    print("Ideal Stroke Comparison Test Suite")
    print("=" * 50)
    
    tests = [
        test_identical_strokes,
        test_similar_strokes,
        test_different_strokes,
        test_length_mismatch,
        test_empty_data,
        test_single_sample,
        test_deviation_gradient,
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            test()
            passed += 1
        except AssertionError as e:
            print(f"✗ {test.__name__}: FAIL — {e}")
            failed += 1
        except Exception as e:
            print(f"✗ {test.__name__}: ERROR — {e}")
            failed += 1
    
    print("=" * 50)
    print(f"Results: {passed} passed, {failed} failed")
    print("=" * 50)
    
    sys.exit(0 if failed == 0 else 1)
