"""Unit tests for geodetic-to-ENU coordinate conversion."""

from fleet_common.geodesy import llh_to_enu

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
