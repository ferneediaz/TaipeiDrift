"""Small, explicit WGS84-to-local-ENU projection for simulated NavSatFix data."""
import math

import numpy as np

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def _ecef(latitude_deg, longitude_deg, altitude_m):
    lat, lon = math.radians(latitude_deg), math.radians(longitude_deg)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    return np.array([(n + altitude_m) * math.cos(lat) * math.cos(lon),
                     (n + altitude_m) * math.cos(lat) * math.sin(lon),
                     (n * (1.0 - WGS84_E2) + altitude_m) * math.sin(lat)])


def geodetic_to_enu(latitude_deg, longitude_deg, altitude_m, origin):
    """Convert WGS84 latitude/longitude/altitude to local east/north/up metres."""
    lat0, lon0, alt0 = origin
    d = _ecef(latitude_deg, longitude_deg, altitude_m) - _ecef(lat0, lon0, alt0)
    lat, lon = math.radians(lat0), math.radians(lon0)
    rot = np.array([[-math.sin(lon), math.cos(lon), 0.0],
                    [-math.sin(lat) * math.cos(lon), -math.sin(lat) * math.sin(lon), math.cos(lat)],
                    [math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat)]])
    # Gazebo city world and the ESKF are both ENU: no NED permutation is applied.
    return rot @ d


def geodetic_covariance_to_enu(covariance, latitude_deg, fallback_sigma_m=(1.5, 1.5, 3.0)):
    """Map NavSatFix's [lat,lon,alt] covariance into ENU, with documented SDF fallback."""
    cov = np.asarray(covariance, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(cov)) or not np.any(np.diag(cov) > 0):
        return np.diag(np.square(fallback_sigma_m))
    lat = math.radians(latitude_deg)
    meridian = WGS84_A * (1.0 - WGS84_E2) / (1.0 - WGS84_E2 * math.sin(lat) ** 2) ** 1.5
    prime_vertical = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    J = np.array([[0.0, prime_vertical * math.cos(lat) * math.pi / 180.0, 0.0],
                  [meridian * math.pi / 180.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0]])
    # ROS NavSatFix covariance orders latitude, longitude, altitude; output is east, north, up.
    P_enu = J @ cov @ J.T
    return 0.5 * (P_enu + P_enu.T)
