"""Unit tests for geodetic-to-ENU coordinate conversion."""

from fleet_common.geodesy import enu_to_llh, enu_to_map, llh_to_enu, map_to_enu

# Datum used in SAGE simulation (forest world origin)
DATUM_LAT = 47.397742
DATUM_LON = 8.545594
DATUM_ALT = 0.0


class TestLlhToEnu:
    def test_datum_returns_origin(self):
        e, n, u = llh_to_enu(DATUM_LAT, DATUM_LON, DATUM_ALT, DATUM_LAT, DATUM_LON, DATUM_ALT)
        assert abs(e) < 0.001
        assert abs(n) < 0.001
        assert abs(u) < 0.001

    def test_north_offset(self):
        lat_offset = DATUM_LAT + 5 * 0.0000090
        e, n, u = llh_to_enu(lat_offset, DATUM_LON, DATUM_ALT, DATUM_LAT, DATUM_LON, DATUM_ALT)
        assert 4.0 < n < 6.0, f"Expected ~5m north, got {n}"
        assert abs(e) < 0.5, f"Expected near-zero east, got {e}"

    def test_east_offset(self):
        lon_offset = DATUM_LON + 5 * 0.0000133
        e, n, u = llh_to_enu(DATUM_LAT, lon_offset, DATUM_ALT, DATUM_LAT, DATUM_LON, DATUM_ALT)
        assert 4.0 < e < 6.0, f"Expected ~5m east, got {e}"
        assert abs(n) < 0.5, f"Expected near-zero north, got {n}"

    def test_enu_to_llh_round_trip(self):
        latitude = DATUM_LAT + 0.000045
        longitude = DATUM_LON + 0.0000665
        east, north, up = llh_to_enu(
            latitude, longitude, 0.0, DATUM_LAT, DATUM_LON, DATUM_ALT
        )
        actual_latitude, actual_longitude, _ = enu_to_llh(
            east, north, up, DATUM_LAT, DATUM_LON, DATUM_ALT
        )
        assert abs(actual_latitude - latitude) < 1e-9
        assert abs(actual_longitude - longitude) < 1e-9


class TestMapEnuRotation:
    def test_zero_offset_preserves_enu_axes(self):
        assert map_to_enu(2.0, 3.0) == (2.0, 3.0)
        assert enu_to_map(2.0, 3.0) == (2.0, 3.0)

    def test_ninety_degree_map_offset_rotates_axes(self):
        from math import pi

        east, north = map_to_enu(1.0, 0.0, pi / 2)
        assert abs(east) < 1e-9
        assert abs(north - 1.0) < 1e-9
        x, y = enu_to_map(east, north, pi / 2)
        assert abs(x - 1.0) < 1e-9
        assert abs(y) < 1e-9
