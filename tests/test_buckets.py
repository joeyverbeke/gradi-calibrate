"""Unit tests for body-centric guidance, the phantom-target wander, and the speech gate.

Run with the venv Python::

    python -m unittest discover -s tests

(No third-party test deps: stdlib unittest only.)
"""

from __future__ import annotations

import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pc_app import buckets, constants  # noqa: E402


def vec(az_deg: float, el_deg: float):
    """Unit (east, north, up) vector from azimuth (atan2(east, north)) / elevation."""
    az = math.radians(az_deg)
    el = math.radians(el_deg)
    cos_el = math.cos(el)
    return (cos_el * math.sin(az), cos_el * math.cos(az), math.sin(el))


NORTH_LEVEL = vec(0.0, 0.0)  # wearer forward: facing north, arm level


class BodyCentricTests(unittest.TestCase):
    def setUp(self) -> None:
        self.state = buckets.GuidanceState()

    def test_far_off_azimuth_high_target_turns(self) -> None:
        # az -60, el 70. Legacy code says "up"; body-centric must turn the body first.
        d = buckets.bucketize_direction(NORTH_LEVEL, vec(-60.0, 70.0), self.state)
        self.assertEqual(d.label, "left")
        self.assertEqual(d.phase, buckets.PHASE_TURN)

    def test_rear_latch_keeps_turn_direction(self) -> None:
        # Directly behind: az wobbles across +-180; the turn direction must not flip.
        first = buckets.bucketize_direction(NORTH_LEVEL, vec(179.5, 45.0), self.state)
        second = buckets.bucketize_direction(NORTH_LEVEL, vec(-179.5, 45.0), self.state)
        self.assertIn(first.label, ("left", "right"))
        self.assertEqual(first.phase, buckets.PHASE_TURN)
        self.assertEqual(first.label, second.label)  # rear latch holds the direction

    def test_zenith_goes_up_immediately(self) -> None:
        # az 0, el 85: no azimuth error -> up, never a TURN.
        d = buckets.bucketize_direction(NORTH_LEVEL, vec(0.0, 85.0), self.state)
        self.assertEqual(d.label, "up")
        self.assertEqual(d.phase, buckets.PHASE_POINT)

    def test_phase_hysteresis_single_transition(self) -> None:
        # Sweep the weighted azimuth error (level target so az_weight == 1) up through
        # the ENTER threshold and back down through EXIT; expect exactly one transition
        # each way and no chatter inside the band.
        up_sweep = [d for d in range(15, 46)]  # 15..45 deg
        transitions = 0
        prev = None
        for az in up_sweep:
            phase = buckets.bucketize_direction(NORTH_LEVEL, vec(float(az), 0.0), self.state).phase
            if prev is not None and phase != prev:
                transitions += 1
            prev = phase
        self.assertEqual(prev, buckets.PHASE_TURN)
        self.assertEqual(transitions, 1)

        transitions = 0
        prev = None
        for az in reversed(up_sweep):
            phase = buckets.bucketize_direction(NORTH_LEVEL, vec(float(az), 0.0), self.state).phase
            if prev is not None and phase != prev:
                transitions += 1
            prev = phase
        self.assertEqual(prev, buckets.PHASE_POINT)
        self.assertEqual(transitions, 1)

        # No chatter: holding a value inside the band keeps the phase stable.
        buckets.bucketize_direction(NORTH_LEVEL, vec(40.0, 0.0), self.state)  # -> TURN
        phases = {
            buckets.bucketize_direction(NORTH_LEVEL, vec(25.0, 0.0), self.state).phase
            for _ in range(5)
        }
        self.assertEqual(phases, {buckets.PHASE_TURN})  # 25 is between EXIT(20) and ENTER(35)

    def test_point_weighted_dominance(self) -> None:
        d_right = buckets.bucketize_direction(NORTH_LEVEL, vec(10.0, 3.0), self.state)
        self.assertEqual(d_right.label, "right")
        self.state.reset()
        d_up = buckets.bucketize_direction(NORTH_LEVEL, vec(3.0, 10.0), self.state)
        self.assertEqual(d_up.label, "up")

    def test_legacy_path_reproduces_old_labels(self) -> None:
        # BODY_CENTRIC_GUIDANCE = False must reproduce the original yaw/pitch labels.
        original = constants.BODY_CENTRIC_GUIDANCE
        buckets.BODY_CENTRIC_GUIDANCE = False
        try:
            # az -60, el 70 -> legacy "up" (the documented regression this rewrite fixes).
            d = buckets.bucketize_direction(NORTH_LEVEL, vec(-60.0, 70.0))
            self.assertEqual(d.label, "up")
            # Simple right-of-forward, level -> "right" in both paths.
            d2 = buckets.bucketize_direction(NORTH_LEVEL, vec(20.0, 2.0))
            self.assertEqual(d2.label, "right")
        finally:
            buckets.BODY_CENTRIC_GUIDANCE = original


class WanderTests(unittest.TestCase):
    def test_offset_magnitude_bounded(self) -> None:
        wander = buckets.TargetWander(rng=random.Random(1234))
        true_vec = vec(30.0, 20.0)
        t = 0.0
        for _ in range(500):
            t += constants.TARGET_BROADCAST_INTERVAL_SEC
            _, (az_off, el_off), _ = wander.step(true_vec, err_deg=1.0, now=t)
            mag = math.hypot(az_off, el_off)
            self.assertGreaterEqual(mag, constants.WANDER_MIN_DEG - 1e-9)
            self.assertLessEqual(mag, constants.WANDER_AMPLITUDE_DEG + 1e-9)

    def test_engagement_scale_bounds(self) -> None:
        self.assertEqual(buckets.wander_engagement_scale(constants.WANDER_ENGAGE_DEG + 1.0), 0.0)
        self.assertEqual(buckets.wander_engagement_scale(constants.NEAR_ENTER_DEG - 1.0), 1.0)

    def test_far_field_target_is_exact(self) -> None:
        # Never engaged (always far): the broadcast target equals the true target.
        wander = buckets.TargetWander(rng=random.Random(7))
        true_vec = vec(30.0, 20.0)
        p_vec, _, scale = wander.step(true_vec, err_deg=50.0, now=2.0)
        self.assertEqual(scale, 0.0)
        for a, b in zip(p_vec, true_vec):
            self.assertAlmostEqual(a, b, places=9)

    def test_engaged_perturbs_target(self) -> None:
        wander = buckets.TargetWander(rng=random.Random(7))
        true_vec = vec(30.0, 20.0)
        # First get close (engages), then the offset should move the broadcast target.
        wander.step(true_vec, err_deg=1.0, now=2.0)
        p_vec, _, scale = wander.step(true_vec, err_deg=1.0, now=4.0)
        self.assertEqual(scale, 1.0)
        sep = math.degrees(math.acos(max(-1.0, min(1.0, sum(a * b for a, b in zip(p_vec, true_vec))))))
        self.assertGreater(sep, 0.1)  # target genuinely moved


if __name__ == "__main__":
    unittest.main()
