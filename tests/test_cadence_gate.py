"""Speech-gate / cadence-stretch tests (plan Workstream B2).

The device ticks fast (DEVICE_TICK_SEC); the host must speak only on the stretched
prompt cadence, stretch it near the target, and never burst prompts after a
playback stall. Run with::

    python -m unittest discover -s tests
"""

from __future__ import annotations

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pc_app import buckets, constants, session  # noqa: E402
from pc_app.planets import ObserverLocation  # noqa: E402


class _StubLink:
    def __init__(self) -> None:
        self.started = []

    def send_session_start(self, *a, **k):
        self.started.append((a, k))

    def send_target_vector(self, *a, **k):
        pass

    def send_session_end(self):
        pass

    def send_idle(self):
        pass

    def read_messages(self):
        return []

    def close(self):
        pass


class _StubAudio:
    should_stream = False

    def __init__(self) -> None:
        self.played = []

    def play_bucket(self, label):
        self.played.append(label)

    def play_intro(self, planet):
        pass

    def play_outro(self):
        pass

    def stop_streaming(self):
        pass


def _decision_with_error(err_deg: float) -> buckets.BucketDecision:
    # Pure elevation error, az_weight 1, so decision_error_deg == err_deg.
    return buckets.BucketDecision(
        label="up" if err_deg >= 0 else "down",
        yaw_error_deg=0.0,
        pitch_error_deg=err_deg,
        phase=buckets.PHASE_POINT,
        d_az_deg=0.0,
        d_el_deg=err_deg,
        az_weight=1.0,
    )


class CadenceGateTests(unittest.TestCase):
    def _controller(self):
        return session.SessionController(
            link=_StubLink(),
            audio=_StubAudio(),
            location=ObserverLocation(latitude_deg=0.0, longitude_deg=0.0),
        )

    def test_fast_ticks_gate_to_prompt_interval(self) -> None:
        ctrl = self._controller()
        ctrl._last_bucket_eval = _decision_with_error(30.0)  # far -> baseline cadence
        clock = {"t": 1000.0}  # monotonic() is large in production; first tick speaks
        spoken = 0
        with mock.patch.object(session.time, "monotonic", lambda: clock["t"]):
            # 20 seconds of ticks at DEVICE_TICK_SEC (0.5s).
            for _ in range(int(20.0 / constants.DEVICE_TICK_SEC)):
                before = len(ctrl._audio.played)
                ctrl._handle_prompt_tick("up")
                spoken += len(ctrl._audio.played) - before
                clock["t"] += constants.DEVICE_TICK_SEC
        # ~20s at 1.5s baseline -> about 13-14 prompts, far fewer than the 40 ticks.
        expected = 20.0 / constants.PROMPT_INTERVAL_SEC
        self.assertLessEqual(abs(spoken - expected), 2)

    def test_interval_stretches_near_target(self) -> None:
        ctrl = self._controller()
        far = ctrl._prompt_interval(constants.NEAR_EXIT_DEG + 5.0)
        self.assertAlmostEqual(far, constants.PROMPT_INTERVAL_SEC)
        # Enter near-mode (err below NEAR_ENTER) then measure the stretch at err -> 0.
        ctrl._update_near_mode(constants.NEAR_ENTER_DEG - 1.0)
        self.assertTrue(ctrl._near_mode)
        near = ctrl._prompt_interval(0.0)
        self.assertAlmostEqual(
            near, constants.PROMPT_INTERVAL_SEC * constants.NEAR_CADENCE_SCALE_MAX
        )
        self.assertGreater(near, far)

    def test_near_mode_hysteresis(self) -> None:
        ctrl = self._controller()
        ctrl._update_near_mode(constants.NEAR_EXIT_DEG - 0.5)  # between ENTER and EXIT
        self.assertFalse(ctrl._near_mode)  # must dip below ENTER to arm
        ctrl._update_near_mode(constants.NEAR_ENTER_DEG - 0.5)
        self.assertTrue(ctrl._near_mode)
        ctrl._update_near_mode(constants.NEAR_ENTER_DEG + 0.5)  # between ENTER and EXIT
        self.assertTrue(ctrl._near_mode)  # stays armed (hysteresis)
        ctrl._update_near_mode(constants.NEAR_EXIT_DEG + 0.5)
        self.assertFalse(ctrl._near_mode)

    def test_no_prompt_burst_after_playback_stall(self) -> None:
        # Ticks queue up during a ~1s clip playback; after the stall at most one prompt
        # plays even though several ticks arrive in the same instant.
        ctrl = self._controller()
        ctrl._last_bucket_eval = _decision_with_error(30.0)
        clock = {"t": 1000.0}
        with mock.patch.object(session.time, "monotonic", lambda: clock["t"]):
            ctrl._handle_prompt_tick("up")  # speaks immediately
            self.assertEqual(len(ctrl._audio.played), 1)
            clock["t"] = 1001.0  # 1s playback stall elapsed
            # Three stale ticks arrive back-to-back at the same wall-clock instant.
            for _ in range(3):
                ctrl._handle_prompt_tick("up")
        # t=1.0 is < PROMPT_INTERVAL_SEC (1.5) after the last prompt -> none of the
        # stale ticks speak; no burst.
        self.assertEqual(len(ctrl._audio.played), 1)


if __name__ == "__main__":
    unittest.main()
