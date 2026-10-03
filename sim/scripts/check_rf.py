"""Check the AIS link model and print its link budget. No simulator needed.

    python3 sim/scripts/check_rf.py [--config sim/config/rf.yaml] [--drone-height 44]

Prints, for each ship class in config/rf.yaml, received power, SNR, bearing spread and packet error rate against
range (no shadowing), and the range where half the packets are lost. Exits non-zero if a check fails.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "nodes"))
import rf_model as rf  # noqa: E402

CONFIG = Path(__file__).resolve().parents[1] / "config" / "rf.yaml"
RANGES_KM = [0.5, 1, 2, 5, 10, 20, 30, 40, 50]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(CONFIG))
    ap.add_argument("--drone-height", type=float, default=44.0, help="m above the sea (demo: 40 m over the pad)")
    args = ap.parse_args()
    cfg = yaml.safe_load(open(args.config))
    ais, prop, rx, df = cfg["ais"], cfg["propagation"], cfg["receiver"], cfg["direction_finder"]
    freq, bits, rb, nf, alpha = ais["channels_hz"][0], ais["packet_bits"], ais["bit_rate"], rx["noise_figure_db"], \
        ais["ber_alpha"]
    impl = rf.calibrate_impl_loss(rx["sensitivity_dbm"], rx["per_at_sensitivity"], bits, rb, nf, alpha)
    floor = rf.noise_floor_dbm(ais["bandwidth_hz"], nf)
    failures = []

    def check(ok, what):
        print(f"  [{'ok' if ok else 'FAIL'}] {what}")
        if not ok:
            failures.append(what)

    def per_at(p_rx):
        return float(rf.packet_error_rate(rf.gmsk_ber(rf.ebn0_db(p_rx, rb, nf, impl), alpha), bits))

    print(f"GMSK {rb} bit/s, BT {ais['bt']}, {ais['bandwidth_hz'] / 1e3:.1f} kHz, {freq / 1e6:.3f} MHz, "
          f"{bits}-bit packets")
    print(f"noise floor {floor:.1f} dBm, implementation loss {impl:.1f} dB "
          f"(so that PER = {rx['per_at_sensitivity']:.0%} at {rx['sensitivity_dbm']:.0f} dBm)\n")

    print("Checks")
    check(abs(per_at(rx["sensitivity_dbm"]) - rx["per_at_sensitivity"]) < 1e-6,
          f"PER at sensitivity is {per_at(rx['sensitivity_dbm']):.3f}")
    check(0 < impl < 15, f"implementation loss {impl:.1f} dB is between 0 and 15 dB")
    # GMSK with BT -> infinity is MSK, whose coherent BER is Q(sqrt(2 Eb/N0)) = 1/2 erfc(sqrt(Eb/N0)) at alpha = 1
    check(abs(float(rf.gmsk_ber(9.6, 1.0)) - 0.5 * math.erfc(math.sqrt(10 ** 0.96))) < 1e-12,
          "GMSK BER with alpha = 1 equals coherent MSK/BPSK")
    check(abs(float(rf.gmsk_ber(9.6, 1.0)) - 1e-5) / 1e-5 < 0.1, "BPSK needs Eb/N0 = 9.6 dB for BER 1e-5")
    d_close = np.linspace(50, 300, 500)
    gap = rf.two_ray_path_loss(d_close, 25, 44, freq, prop["sea_reflection"]) - rf.free_space_path_loss(d_close, freq)
    check(gap.min() > -6.0, f"close in, two-ray is never more than 6 dB better than free space ({gap.min():.1f} dB)")
    h_tx, h_rx, d = 25.0, 44.0, 30_000.0
    flat = 40 * math.log10(d) - 20 * math.log10(h_tx * h_rx)
    two = float(rf.two_ray_path_loss(d, h_tx, h_rx, freq, -1.0))
    check(abs(two - flat) < 0.5, f"far out, two-ray is 40 log d - 20 log(ht hr) ({two:.1f} vs {flat:.1f} dB)")

    print()
    for cls, h_tx in (("A", 25.0), ("B", 6.0)):
        tx = ais["class_power_dbm"][cls]
        horizon = rf.radio_horizon_m(h_tx, args.drone_height)
        print(f"Class {cls}: {tx:.0f} dBm, antenna {h_tx:.0f} m, drone at {args.drone_height:.0f} m, "
              f"radio horizon {horizon / 1e3:.1f} km")
        print("   range km   P_rx dBm   SNR dB   bearing sigma deg   PER")
        for km in RANGES_KM:
            p = rf.link(km * 1e3, h_tx, args.drone_height, freq, tx, 2.15, 2.0, rx["antenna_gain_db"], -1.0)
            if not math.isfinite(p):
                print(f"   {km:8.1f}   beyond the radio horizon")
                continue
            sig = math.degrees(rf.bearing_sigma_rad(p - floor, df["sigma_floor_deg"], df["sigma_at_ref_deg"],
                                                    df["snr_ref_db"]))
            print(f"   {km:8.1f}   {p:8.1f}   {p - floor:6.1f}   {sig:17.1f}   {per_at(p):5.3f}")
        # half the packets lost: walk out until PER >= 0.5 (smooth past the last lobe, so fine in 100 m steps)
        d50 = next((d for d in np.arange(5_000, 100_000, 100.0)
                    if per_at(rf.link(d, h_tx, args.drone_height, freq, tx, 2.15, 2.0, rx["antenna_gain_db"], -1.0)) >= 0.5),
                   None)
        print(f"   50 % packet loss at {d50 / 1e3:.1f} km\n" if d50 else "   decoded out to 100 km\n")
        if cls == "A":
            check(d50 is not None and 15_000 < d50 <= horizon + 100,
                  "Class A is heard over tens of km, up to the radio horizon")
        print()

    print("All checks passed" if not failures else f"{len(failures)} check(s) failed")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
