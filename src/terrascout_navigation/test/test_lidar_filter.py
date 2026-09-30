"""Unit tests for lidar_filter point-stripping logic (no ROS required)."""

import struct

import numpy as np

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_cloud_data(points: list[tuple[float, float, float]]) -> tuple[bytes, int]:
    """Build minimal PointCloud2 data buffer from (x,y,z) tuples.

    Returns (data_bytes, point_step).  Uses float32 xyz at offsets 0,4,8
    with point_step=12 (no extra fields).
    """
    step = 12
    buf = bytearray()
    for x, y, z in points:
        buf.extend(struct.pack("<fff", x, y, z))
    return bytes(buf), step


# Livox Mid-360 format: x(f32) y(f32) z(f32) intensity(f32) tag(u8) line(u8) timestamp(f64)
_LIVOX_STEP = 26


def _make_livox_data(
    points: list[tuple[float, float, float, int]],
) -> tuple[bytes, int]:
    """Build Livox-format PointCloud2 buffer from (x, y, z, tag) tuples.

    Intensity, line, and timestamp are zero-filled.
    """
    buf = bytearray()
    for x, y, z, tag in points:
        buf.extend(struct.pack("<ffff", x, y, z, 0.0))  # x,y,z,intensity
        buf.extend(struct.pack("<BB", tag, 0))  # tag, line
        buf.extend(struct.pack("<d", 0.0))  # timestamp
    return bytes(buf), _LIVOX_STEP


def _filter_points(data: bytes, point_step: int, **kwargs) -> np.ndarray:
    """Import and run the core filter logic."""
    from terrascout_navigation.lidar_filter import _valid_mask

    return _valid_mask(data, point_step, **kwargs)


# ---------------------------------------------------------------------------
# Original tests (basic validity)
# ---------------------------------------------------------------------------


class TestBasicValidity:
    def test_keeps_normal_points(self):
        data, step = _make_cloud_data([(5.0, 2.0, 3.0), (4.0, 5.0, 6.0)])
        mask = _filter_points(data, step, min_range_sq=0.0)
        assert mask.sum() == 2

    def test_removes_zero_points(self):
        data, step = _make_cloud_data([(0.0, 0.0, 0.0), (1.0, 2.0, 3.0)])
        mask = _filter_points(data, step, min_range_sq=0.0)
        assert mask.sum() == 1

    def test_removes_inf_points(self):
        data, step = _make_cloud_data([(float("inf"), 0.0, 0.0), (1.0, 2.0, 3.0)])
        mask = _filter_points(data, step, min_range_sq=0.0)
        assert mask.sum() == 1

    def test_removes_nan_points(self):
        data, step = _make_cloud_data([(float("nan"), 1.0, 2.0), (1.0, 2.0, 3.0)])
        mask = _filter_points(data, step, min_range_sq=0.0)
        assert mask.sum() == 1

    def test_empty_cloud(self):
        mask = _filter_points(b"", 12)
        assert len(mask) == 0

    def test_all_bad_points(self):
        data, step = _make_cloud_data(
            [
                (0.0, 0.0, 0.0),
                (float("inf"), 0.0, 0.0),
                (float("nan"), 1.0, 2.0),
            ]
        )
        mask = _filter_points(data, step, min_range_sq=0.0)
        assert mask.sum() == 0


# ---------------------------------------------------------------------------
# Blind-zone / minimum range tests
# ---------------------------------------------------------------------------


class TestMinRange:
    def test_removes_near_origin_noise(self):
        """Near-zero points like (0.001, 0, 0) are blind-zone noise."""
        data, step = _make_cloud_data([(0.001, 0.0, 0.0)])
        mask = _filter_points(data, step)  # default min_range_sq=0.25 (0.5m)
        assert mask.sum() == 0

    def test_keeps_point_beyond_blind_zone(self):
        data, step = _make_cloud_data([(1.0, 0.0, 0.0)])
        mask = _filter_points(data, step)
        assert mask.sum() == 1

    def test_boundary_exactly_at_blind_zone(self):
        """Point at exactly 0.5m should be excluded (> not >=)."""
        data, step = _make_cloud_data([(0.5, 0.0, 0.0)])
        mask = _filter_points(data, step)  # 0.5^2 = 0.25 == min_range_sq, not >
        assert mask.sum() == 0

    def test_point_just_beyond_blind_zone(self):
        data, step = _make_cloud_data([(0.51, 0.0, 0.0)])
        mask = _filter_points(data, step)
        assert mask.sum() == 1

    def test_custom_blind_zone(self):
        """With min_range_sq=0.01 (0.1m), a point at 0.2m passes."""
        data, step = _make_cloud_data([(0.2, 0.0, 0.0)])
        mask = _filter_points(data, step, min_range_sq=0.01)
        assert mask.sum() == 1

    def test_disabled_blind_zone(self):
        """With min_range_sq=0.0, even near-zero passes (if non-zero)."""
        data, step = _make_cloud_data([(0.001, 0.0, 0.0)])
        mask = _filter_points(data, step, min_range_sq=0.0)
        assert mask.sum() == 1


# ---------------------------------------------------------------------------
# Livox tag-based noise filtering
# ---------------------------------------------------------------------------


class TestLivoxTagFilter:
    def test_keeps_clean_livox_points(self):
        data, step = _make_livox_data(
            [
                (5.0, 0.0, 0.0, 0),  # tag=0 → normal
                (0.0, 3.0, 0.0, 0),
            ]
        )
        mask = _filter_points(data, step, min_range_sq=0.0)
        assert mask.sum() == 2

    def test_removes_spatial_noise(self):
        """Tag bits 0-1 non-zero → spatial noise."""
        data, step = _make_livox_data(
            [
                (5.0, 0.0, 0.0, 1),  # tag=1 → spatial noise level 1
                (5.0, 0.0, 0.0, 2),  # tag=2 → spatial noise level 2
            ]
        )
        mask = _filter_points(data, step, min_range_sq=0.0)
        assert mask.sum() == 0

    def test_removes_intensity_noise(self):
        """Tag bits 2-3 non-zero → intensity noise (dust/rain/fog)."""
        data, step = _make_livox_data(
            [
                (5.0, 0.0, 0.0, 4),  # tag=4 → intensity noise (dust)
                (5.0, 0.0, 0.0, 8),  # tag=8 → intensity noise (rain/fog)
            ]
        )
        mask = _filter_points(data, step, min_range_sq=0.0)
        assert mask.sum() == 0

    def test_removes_dragging_noise(self):
        """Tag bits 4-5 non-zero → near-lidar dragging noise (issue #55)."""
        data, step = _make_livox_data(
            [
                (5.0, 0.0, 0.0, 16),  # tag=16 → dragging noise level 1
                (5.0, 0.0, 0.0, 32),  # tag=32 → dragging noise level 2
            ]
        )
        mask = _filter_points(data, step, min_range_sq=0.0)
        assert mask.sum() == 0

    def test_keeps_points_when_tag_filter_disabled(self):
        """With filter_noise_tag=False, noisy tags are ignored."""
        data, step = _make_livox_data(
            [
                (5.0, 0.0, 0.0, 16),
                (5.0, 0.0, 0.0, 4),
            ]
        )
        mask = _filter_points(data, step, min_range_sq=0.0, filter_noise_tag=False)
        assert mask.sum() == 2

    def test_combined_noise_and_blind_zone(self):
        """Both filters active: only clean far points survive."""
        data, step = _make_livox_data(
            [
                (5.0, 0.0, 0.0, 0),  # clean + far → keep
                (0.1, 0.0, 0.0, 0),  # clean + close → reject (blind zone)
                (5.0, 0.0, 0.0, 16),  # noisy + far → reject (tag)
                (0.1, 0.0, 0.0, 4),  # noisy + close → reject (both)
            ]
        )
        mask = _filter_points(data, step)
        assert mask.sum() == 1

    def test_no_tag_filter_on_short_point_step(self):
        """Generic PointCloud2 (point_step=12) has no tag — skip tag filter."""
        data, step = _make_cloud_data([(5.0, 0.0, 0.0)])
        # This should work fine even though there's no tag field
        mask = _filter_points(data, step, min_range_sq=0.0)
        assert mask.sum() == 1
