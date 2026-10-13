from functools import lru_cache

from pyproj import Transformer
from pyproj.enums import TransformDirection


def llh_to_enu(
    latitude: float,
    longitude: float,
    altitude: float,
    datum_latitude: float,
    datum_longitude: float,
    datum_altitude: float,
) -> tuple[float, float, float]:
    return _to_enu(datum_latitude, datum_longitude, datum_altitude).transform(
        longitude, latitude, altitude
    )


def enu_to_llh(
    east: float,
    north: float,
    up: float,
    datum_latitude: float,
    datum_longitude: float,
    datum_altitude: float,
) -> tuple[float, float, float]:
    longitude, latitude, altitude = _to_enu(
        datum_latitude, datum_longitude, datum_altitude
    ).transform(east, north, up, direction=TransformDirection.INVERSE)
    return latitude, longitude, altitude


@lru_cache(maxsize=4)
def _to_enu(latitude: float, longitude: float, altitude: float) -> Transformer:
    return Transformer.from_pipeline(
        "+proj=pipeline +step +proj=cart +ellps=WGS84 "
        f"+step +proj=topocentric +ellps=WGS84 +lat_0={latitude!r} +lon_0={longitude!r} +h_0={altitude!r}"
    )
