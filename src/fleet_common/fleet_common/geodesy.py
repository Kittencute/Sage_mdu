from functools import lru_cache

from pyproj import Transformer


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


@lru_cache(maxsize=4)
def _to_enu(latitude: float, longitude: float, altitude: float) -> Transformer:
    return Transformer.from_pipeline(
        "+proj=pipeline +step +proj=cart +ellps=WGS84 "
        f"+step +proj=topocentric +ellps=WGS84 +lat_0={latitude!r} +lon_0={longitude!r} +h_0={altitude!r}"
    )
