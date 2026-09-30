import math

import pytest

from madum_bridge.command_handlers import (
    parse_camera_image_params,
    parse_nav_waypoint_params,
    parse_video_start_params,
    parse_video_stop_params,
)


def test_parse_nav_waypoint_params():
    params = [20.0, 1.5, 1.0, 90.0, 65.575422, 19.165765, 50.0]
    result = parse_nav_waypoint_params(params)
    assert result["hold_time_s"] == 2.0
    assert result["acceptance_radius_m"] == 1.5
    assert result["mission_related"] is True
    assert result["desired_yaw_deg"] == 90.0
    assert result["latitude"] == 65.575422
    assert result["longitude"] == 19.165765


def test_parse_nav_waypoint_pass_through():
    params = [0.0, 0.0, 0.0, float("nan"), 65.575, 19.165, 50.0]
    result = parse_nav_waypoint_params(params)
    assert result["hold_time_s"] == 0.0
    assert result["mission_related"] is False
    assert math.isnan(result["desired_yaw_deg"])


def test_parse_camera_image_params():
    params = [45.0, -30.0, 3.0, 2.0]
    result = parse_camera_image_params(params)
    assert result["yaw_deg"] == 45.0
    assert result["pitch_deg"] == -30.0
    assert result["photo_count"] == 3
    assert result["interval_s"] == 2.0


def test_parse_camera_image_defaults():
    params = [0.0, 0.0, 1.0, 0.0]
    result = parse_camera_image_params(params)
    assert result["photo_count"] == 1
    assert result["interval_s"] == 0.0


def test_parse_video_start_params():
    params = [0.0, 30.0]
    result = parse_video_start_params(params)
    assert result["stream_id"] == 0
    assert result["frequency_hz"] == 30.0


def test_parse_video_stop_params():
    params = [1.0]
    result = parse_video_stop_params(params)
    assert result["stream_id"] == 1


def test_nav_waypoint_short_params():
    with pytest.raises(ValueError, match="NAV_WAYPOINT requires 6 params, got 3"):
        parse_nav_waypoint_params([0.0, 0.0, 0.0])


def test_camera_image_short_params():
    with pytest.raises(ValueError, match="CAMERA_IMAGE requires 4 params, got 1"):
        parse_camera_image_params([0.0])


def test_video_start_short_params():
    with pytest.raises(ValueError, match="VIDEO_START_CAPTURE requires 2 params, got 0"):
        parse_video_start_params([])


def test_video_stop_short_params():
    with pytest.raises(ValueError, match="VIDEO_STOP_CAPTURE requires 1 params, got 0"):
        parse_video_stop_params([])
