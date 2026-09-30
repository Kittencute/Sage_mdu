"""Unit tests for teleop_takeover rising-edge logic (no ROS required)."""

from terrascout_navigation.teleop_takeover import detect_rising_edge


class TestRisingEdge:
    def test_false_to_false(self):
        assert detect_rising_edge(False, False) is False

    def test_false_to_true_is_rising_edge(self):
        assert detect_rising_edge(False, True) is True

    def test_true_to_true_held(self):
        """Held button must NOT re-trigger — one cancel per press."""
        assert detect_rising_edge(True, True) is False

    def test_true_to_false(self):
        assert detect_rising_edge(True, False) is False

    def test_full_press_release_cycle(self):
        """Simulate the state machine over a sequence of joy messages."""
        prev = False
        triggers = 0
        # Sequence: idle, press, hold, hold, release, idle, press
        for curr in [False, True, True, True, False, False, True]:
            if detect_rising_edge(prev, curr):
                triggers += 1
            prev = curr
        # Two distinct presses → two cancels
        assert triggers == 2
