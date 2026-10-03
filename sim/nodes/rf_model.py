"""Radio link and direction-finding model for AIS (GMSK) transmitters on ships, received by a drone.

Pure functions, no ROS, so sim/scripts/check_rf.py can test them. Used by sim/nodes/rf_sensor.py.

Per packet, the chain is the one Gazebo's RFComms uses (path loss, shadowing, bit error rate, packet error rate),
with two of its errors fixed. RFComms treats the received-to-noise power ratio as Eb/N0, and raises the bit error
rate to the number of bytes instead of bits.
    P_rx   = P_tx + G_tx - L_cable - PL_two_ray(d, h_tx, h_rx) + N(0, sigma_shadow) + G_rx       [dBm]
    Eb/N0  = P_rx - (-174 + NF + 10 log10 Rb) - L_impl                                         [dB]
    BER    = 1/2 erfc( sqrt(alpha Eb/N0) )                 GMSK (Rappaport, eq. 6.115 form)
    PER    = 1 - (1 - BER)^bits
L_impl stands for everything a real receiver loses against the ideal coherent detector: discriminator detection,
interference, noise from the drone's own motors. It is not set by hand. calibrate_impl_loss() solves it so the
receiver meets its sensitivity spec (IEC 61993-2: 20 % packet error rate at the sensitivity level).
"""
import math
import xml.etree.ElementTree as ET

import numpy as np
from scipy.special import erfc, erfcinv

C = 299_792_458.0          # m/s
KT_DBM_HZ = -174.0         # thermal noise density at 290 K, dBm/Hz
KNOT = 1852.0 / 3600.0     # m/s


def wavelength(freq_hz: float) -> float:
    return C / freq_hz


def free_space_path_loss(d_m, freq_hz):
    """Friis free-space loss in dB."""
    return 20 * np.log10(4 * math.pi * np.maximum(d_m, 1e-3) / wavelength(freq_hz))


def two_ray_path_loss(d_m, h_tx, h_rx, freq_hz, reflection=-0.9):
    """Path loss in dB over a flat sea: the direct ray plus the ray reflected off the water.

    d_m is the horizontal distance; h_tx, h_rx are antenna heights above the sea. The calm sea at VHF and
    grazing angles reflects with a coefficient close to -1. Close in, the two rays beat against each other
    (lobes and nulls around free space); beyond d_c = 4 pi h_tx h_rx / lambda the loss grows as 40 log10(d).
    """
    lam = wavelength(freq_hz)
    k = 2 * math.pi / lam
    r1 = np.hypot(d_m, h_tx - h_rx)
    r2 = np.hypot(d_m, h_tx + h_rx)
    field = np.exp(-1j * k * r1) / r1 + reflection * np.exp(-1j * k * r2) / r2
    gain = (lam / (4 * math.pi)) ** 2 * np.abs(field) ** 2
    return -10 * np.log10(np.maximum(gain, 1e-30))


def radio_horizon_m(h_tx, h_rx):
    """Radio horizon over a smooth earth with standard refraction (k = 4/3): 4.12 (sqrt h_tx + sqrt h_rx) km."""
    return 4120.0 * (math.sqrt(max(h_tx, 0.0)) + math.sqrt(max(h_rx, 0.0)))


def noise_floor_dbm(bandwidth_hz, noise_figure_db):
    """Receiver noise power in the channel bandwidth."""
    return KT_DBM_HZ + 10 * math.log10(bandwidth_hz) + noise_figure_db


def ebn0_db(p_rx_dbm, bit_rate, noise_figure_db, impl_loss_db):
    """Energy per bit over noise density: Eb/N0 = P_rx / (Rb k T F), less the implementation loss."""
    return p_rx_dbm - (KT_DBM_HZ + noise_figure_db + 10 * math.log10(bit_rate)) - impl_loss_db


def gmsk_ber(ebn0_db_, alpha):
    """GMSK bit error rate, Pb = Q(sqrt(2 alpha Eb/N0)) = 1/2 erfc(sqrt(alpha Eb/N0)).

    alpha depends on BT: 0.68 at BT = 0.25, 0.85 for plain MSK (Rappaport, Wireless Communications, 6.9.4).
    """
    return 0.5 * erfc(np.sqrt(alpha * 10 ** (np.asarray(ebn0_db_) / 10)))


def packet_error_rate(ber, bits):
    """A packet is lost if any of its bits is wrong (AIS has a CRC and no error correction)."""
    return 1.0 - (1.0 - np.asarray(ber)) ** bits


def calibrate_impl_loss(sensitivity_dbm, per_at_sensitivity, bits, bit_rate, noise_figure_db, alpha):
    """Implementation loss (dB) that makes PER = per_at_sensitivity at P_rx = sensitivity_dbm."""
    ber = 1.0 - (1.0 - per_at_sensitivity) ** (1.0 / bits)
    ebn0_needed_db = 10 * math.log10(erfcinv(2 * ber) ** 2 / alpha)
    return ebn0_db(sensitivity_dbm, bit_rate, noise_figure_db, 0.0) - ebn0_needed_db


def bearing_sigma_rad(snr_db, floor_deg, at_ref_deg, snr_ref_db):
    """Spread of a direction-finding bearing: a floor (array calibration, multipath off the hull and the sea)
    plus a noise term that falls by half for every 6 dB of SNR, as the Cramer-Rao bound does."""
    noise = at_ref_deg * 10 ** (-(snr_db - snr_ref_db) / 20)
    return math.radians(math.hypot(floor_deg, noise))


def ais_report_interval_s(ais_class, speed_mps):
    """Nominal reporting interval (ITU-R M.1371-5, Annex 1, Tables 1 and 2), ignoring the faster rate when
    turning. Class A: 10 s up to 14 knots, 6 s up to 23 knots, 2 s above, 3 min at anchor.
    Class B "CS": 30 s above 2 knots, 3 min below."""
    kn = speed_mps / KNOT
    if ais_class.upper() == "A":
        if kn < 0.5:
            return 180.0
        return 10.0 if kn <= 14 else 6.0 if kn <= 23 else 2.0
    return 30.0 if kn > 2 else 180.0


def link(d_horizontal, h_tx, h_rx, freq_hz, tx_dbm, tx_gain_db, cable_loss_db, rx_gain_db, reflection,
         shadow_db=0.0):
    """Received power in dBm before any receiver noise, or -inf beyond the radio horizon."""
    if d_horizontal > radio_horizon_m(h_tx, h_rx):
        return -math.inf
    pl = float(two_ray_path_loss(d_horizontal, h_tx, h_rx, freq_hz, reflection))
    return tx_dbm + tx_gain_db - cable_loss_db - pl + shadow_db + rx_gain_db


# Positions in AIS messages are latitude and longitude. Over a few km a local tangent plane at the world origin is
# exact to well under a metre (WGS84 radii of curvature at the origin latitude).
WGS84_A, WGS84_E2 = 6378137.0, 6.69437999014e-3


def _radii(lat0_deg):
    s = math.sin(math.radians(lat0_deg)) ** 2
    meridian = WGS84_A * (1 - WGS84_E2) / (1 - WGS84_E2 * s) ** 1.5
    normal = WGS84_A / math.sqrt(1 - WGS84_E2 * s)
    return meridian, normal * math.cos(math.radians(lat0_deg))


def enu_to_latlon(x, y, lat0_deg, lon0_deg):
    """World metres east, north of the origin to degrees."""
    m_north, m_east = _radii(lat0_deg)
    return lat0_deg + math.degrees(y / m_north), lon0_deg + math.degrees(x / m_east)


def latlon_to_enu(lat_deg, lon_deg, lat0_deg, lon0_deg):
    """Degrees to world metres east, north of the origin."""
    m_north, m_east = _radii(lat0_deg)
    return math.radians(lon_deg - lon0_deg) * m_east, math.radians(lat_deg - lat0_deg) * m_north


def world_origin(sdf_path):
    """Latitude and longitude of a world's origin, from its <spherical_coordinates>."""
    sc = ET.parse(sdf_path).find(".//spherical_coordinates")
    return float(sc.findtext("latitude_deg")), float(sc.findtext("longitude_deg"))
