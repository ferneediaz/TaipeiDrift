"""Benchmark harness: discovery, incomplete data, NaN-safe aggregation, pairing, resume, determinism, no leakage."""
import json
import os
import zipfile
from pathlib import Path

import cv2
import h5py
import numpy as np
import pytest
import yaml

from src.data.synthetic import ImuNoise, make_synthetic_trajectory
from vio.benchmark import aggregate as agg
from vio.benchmark.discovery import discover
from vio.benchmark.runner import run_trajectory, series_metrics

REPO = Path(__file__).resolve().parents[2]
CUT = 5.0


def _write_flight(f: h5py.File, name: str, duration: float, drop: str | None = None, gt_offset_after: float = 0.0,
                  frames: int | None = None):
    tr = make_synthetic_trajectory("circle", duration=duration, gyroscope_frame="world",
                                   noise=ImuNoise(gyro_bias=(0.002, -0.001, 0.001), gyro_white_std=0.01, seed=1))
    pos = tr.position_gt.copy()
    if gt_offset_after:
        pos[int(CUT * 100) + 1:] += gt_offset_after
    data = {"imu/accelerometer": tr.accelerometer, "imu/gyroscope": tr.gyroscope, "groundtruth/position": pos,
            "groundtruth/velocity": tr.velocity_gt, "groundtruth/attitude": tr.attitude_gt}
    g = f.create_group(name)
    for k, v in data.items():
        if k != drop:
            g.create_dataset(k, data=v)
    n_frames = frames if frames is not None else len(tr) // 4 + 1
    g.create_dataset("camera_data/color_left", data=np.array([f"color_left/{name}/{i:06d}.JPEG".encode() for i in range(n_frames)]))
    return n_frames


def _write_frames(path: Path, n: int, skip: int | None = None):
    path.parent.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(0)
    base = (rng.random((64, 64)) * 255).astype(np.uint8)
    with zipfile.ZipFile(path, "w") as z:
        for i in range(n):
            if i == skip:
                continue
            img = np.roll(base, i % 7, axis=1)
            z.writestr(f"{i:06d}.JPEG", cv2.imencode(".jpg", img)[1].tobytes())


def make_dataset(root: Path, gt_offset_after: float = 0.0) -> Path:
    """Fake Mid-Air folder: one good condition (zipped sensor file) and one with a truncated zip."""
    cond = root / "Kite_training" / "sunny"
    cond.mkdir(parents=True)
    h5 = root / "tmp_sensor.hdf5"
    with h5py.File(h5, "w") as f:
        n0 = _write_flight(f, "trajectory_0000", 20.0, gt_offset_after=gt_offset_after)  # full, with images
        _write_flight(f, "trajectory_0001", 20.0)  # no images downloaded
        n2 = _write_flight(f, "trajectory_0002", 20.0)  # corrupt frames.zip
        _write_flight(f, "trajectory_0003", 5.0)  # too short
        _write_flight(f, "trajectory_0004", 20.0, drop="groundtruth/velocity")  # missing dataset
        n5 = _write_flight(f, "trajectory_0005", 20.0)  # a listed frame missing from the archive
    with zipfile.ZipFile(cond / "sensor_records.zip", "w", zipfile.ZIP_DEFLATED) as z:
        z.write(h5, "sensor_records.hdf5")
    h5.unlink()
    _write_frames(cond / "color_left" / "trajectory_0000" / "frames.zip", n0)
    bad = cond / "color_left" / "trajectory_0002" / "frames.zip"
    _write_frames(bad, n2)
    bad.write_bytes(bad.read_bytes()[: len(bad.read_bytes()) // 2])  # truncated: still downloading
    _write_frames(cond / "color_left" / "trajectory_0005" / "frames.zip", n5, skip=10)
    # second condition: sensor zip cut off mid-download
    foggy = root / "Kite_training" / "foggy"
    foggy.mkdir(parents=True)
    (foggy / "sensor_records.zip").write_bytes((cond / "sensor_records.zip").read_bytes()[:5000])
    (foggy / "color_left" / "trajectory_2000").mkdir(parents=True)
    return root


def _snapshot(root: Path) -> dict:
    return {str(p.relative_to(root)): (p.stat().st_size, p.stat().st_mtime_ns) for p in root.rglob("*") if p.is_file()}


@pytest.fixture
def dataset(tmp_path):
    return make_dataset(tmp_path / "MidAir")


# ------------------------------------------------------------- discovery
def test_discovery_classifies_every_trajectory(dataset, tmp_path):
    r = discover(dataset, tmp_path / "cache")
    by = {e.trajectory: e for e in r.entries}
    assert set(by) == {f"trajectory_000{i}" for i in range(6)}
    assert by["trajectory_0000"].imu_valid and by["trajectory_0000"].visual_valid
    assert by["trajectory_0001"].imu_valid and not by["trajectory_0001"].visual_valid
    assert "not downloaded" in by["trajectory_0001"].visual_reason
    assert by["trajectory_0002"].imu_valid and "unreadable" in by["trajectory_0002"].visual_reason
    assert not by["trajectory_0003"].imu_valid and "too short" in by["trajectory_0003"].imu_reason
    assert not by["trajectory_0004"].imu_valid and "missing datasets" in by["trajectory_0004"].imu_reason
    assert by["trajectory_0005"].imu_valid and "missing from frames.zip" in by["trajectory_0005"].visual_reason
    c = r.counts()
    assert (c["imu_valid"], c["visual_valid"], c["conditions_unusable"]) == (4, 1, 1)
    assert c["discovered"] == 7  # 6 flights + 1 camera folder in the unusable condition


def test_incomplete_sensor_zip_is_reported_not_fatal(dataset, tmp_path):
    r = discover(dataset, tmp_path / "cache")
    (issue,) = r.condition_issues
    assert issue["condition"] == "foggy" and "unreadable" in issue["reason"]


def test_dataset_folder_is_never_modified(dataset, tmp_path):
    before = _snapshot(dataset)
    r = discover(dataset, tmp_path / "cache")
    assert _snapshot(dataset) == before
    assert Path(r.entries[0].sensor_file).is_relative_to(tmp_path / "cache")


def test_filters(dataset, tmp_path):
    r = discover(dataset, tmp_path / "cache", conditions=["sunny"], trajectories=["1", "trajectory_0005"])
    assert [e.trajectory for e in r.entries] == ["trajectory_0001", "trajectory_0005"]
    assert not r.condition_issues


# ------------------------------------------------------------- metrics and aggregation
def test_short_trajectory_gives_nan_beyond_its_end():
    t = np.linspace(0, 25, 2501)
    m = series_metrics(t, t * 2.0, [10, 30, 60])
    assert m["10s"] == pytest.approx(20.0)
    assert np.isnan(m["30s"]) and np.isnan(m["60s"])
    assert m["final"] == pytest.approx(50.0)


def test_describe_ignores_nan_and_none():
    d = agg.describe([1.0, float("nan"), None, 3.0, 2.0])
    assert d["n"] == 3 and d["median"] == 2.0 and d["min"] == 1.0 and d["max"] == 3.0
    assert agg.describe([float("nan")])["n"] == 0


def _res(key, imu_p, vis_p, imu_a, vis_a, orc_p=10.0, valid=0.9):
    m = lambda p, a: {"position_m": {"final": p, "10s": p / 10}, "attitude_deg": {"final": a, "10s": a / 10}}  # noqa: E731
    return {"key": key, "condition": "sunny", "methods": {"imu_only": m(imu_p, imu_a),
                                                          "visual": m(vis_p, vis_a) if vis_p is not None else None,
                                                          "ground_truth_attitude_oracle": m(orc_p, 0.0)},
            "visual_statistics": {"valid_fraction": valid} if vis_p is not None else None}


def test_paired_classification_and_counts():
    tol = {"position_m": {"abs": 1.0, "rel": 0.02}, "attitude_deg": {"abs": 0.05, "rel": 0.02}}
    den = {"position_m": 5.0, "attitude_deg": 0.5}
    results = [_res("a", 100, 50, 6.0, 3.0), _res("b", 100, 150, 1.0, 2.0), _res("c", 100, 100.5, 0.2, 0.21),
               _res("d", 100, None, 1.0, None)]
    p = agg.paired(results, ["final"], tol, den)
    pos, att = p["position_m"]["final"], p["attitude_deg"]["final"]
    assert pos["n_pairs"] == 3 and (pos["improved"], pos["worsened"], pos["unchanged"]) == (1, 1, 1)
    assert pos["median_percent_change"] == pytest.approx(0.5)
    assert att["improved"] == 1 and att["worsened"] == 1 and att["unchanged"] == 1
    assert att["n_percent_defined"] == 2  # 0.2 deg is below the 0.5 deg denominator floor


def test_classify_tolerance():
    assert agg.classify(-0.5, 100.0, 1.0, 0.02) == "unchanged"
    assert agg.classify(-3.0, 100.0, 1.0, 0.02) == "improved"
    assert agg.classify(3.0, 100.0, 1.0, 0.02) == "worsened"
    assert agg.classify(float("nan"), 1.0, 1.0, 0.02) == "unavailable"


def test_drift_strata_tertiles_and_thresholds():
    tol = {"position_m": {"abs": 1.0, "rel": 0.02}, "attitude_deg": {"abs": 0.05, "rel": 0.02}}
    results = [_res(str(i), 100, 90, a, a * 0.5 if a > 3 else a * 1.5) for i, a in enumerate([0.5, 0.8, 1.0, 3.5, 6.0, 9.0])]
    s = agg.drift_strata(results, {"thresholds_deg": None}, tol)
    assert sum(g["n"] for g in s["groups"].values()) == 6
    assert s["groups"]["high"]["attitude_deg"]["improved"] == 2 and s["groups"]["low"]["attitude_deg"]["worsened"] == 2
    s2 = agg.drift_strata(results, {"thresholds_deg": [1.0, 5.0]}, tol)
    assert [s2["groups"][g]["n"] for g in ("low", "medium", "high")] == [2, 2, 2]


def test_oracle_gap_and_small_denominators():
    g = agg.oracle_gap([_res("a", 100, 40, 1, 1, orc_p=10), _res("b", 3, 2, 1, 1, orc_p=1)], min_denominator_m=5.0)
    rows = {r["key"]: r for r in g["per_trajectory"]}
    assert rows["a"]["imu_minus_oracle_m"] == 90 and rows["a"]["visual_minus_oracle_m"] == 30
    assert rows["a"]["oracle_reduction_vs_imu_percent"] == pytest.approx(90.0)
    assert rows["a"]["visual_share_of_gap_closed_percent"] == pytest.approx(100 * 60 / 90)
    assert np.isnan(rows["b"]["oracle_reduction_vs_imu_percent"])  # 3 m IMU error: below the floor


# ------------------------------------------------------------- end to end, resume, determinism
def _args(dataset, out, *extra):
    from vio.scripts.run_midair_benchmark import parse_args
    cfg = yaml.safe_load((REPO / "vio/configs/midair_benchmark.yaml").read_text())
    cfg["horizons_s"] = [10, 30, 60]
    cpath = out.parent / "bench.yaml"
    cpath.write_text(yaml.safe_dump(cfg))
    return parse_args(["--config", str(cpath), "--data-root", str(dataset), "--output-dir", str(out), *extra])


def test_end_to_end_resume_and_deterministic_output(dataset, tmp_path):
    from vio.scripts.run_midair_benchmark import run_benchmark
    out = tmp_path / "bench"
    s1 = run_benchmark(_args(dataset, out, "--resume"), make_plots=False)
    assert s1["n_trajectories"] == 4 and s1["n_with_visual"] == 1
    csv1 = (out / "trajectory_metrics.csv").read_bytes()
    m0 = out / "trajectories" / "Kite_training_sunny_trajectory_0000" / "metrics.json"
    stamp = m0.stat().st_mtime_ns
    d = json.loads(m0.read_text())
    assert np.isnan(d["methods"]["imu_only"]["position_m"].get("30s") or float("nan"))  # 15 s after the cutoff only
    assert d["methods"]["ground_truth_attitude_oracle"]["attitude_deg"]["max"] < 1e-6

    run_benchmark(_args(dataset, out, "--resume"), make_plots=False)
    assert m0.stat().st_mtime_ns == stamp  # reused, not recomputed
    assert (out / "trajectory_metrics.csv").read_bytes() == csv1

    run_benchmark(_args(dataset, out, "--overwrite"), make_plots=False)
    assert m0.stat().st_mtime_ns != stamp
    assert (out / "trajectory_metrics.csv").read_bytes() == csv1  # same numbers, same formatting

    rows = (out / "trajectory_metrics.csv").read_text().splitlines()
    assert len(rows) == 1 + 4 * 3
    assert any(",visual," in r and "unavailable: no color_left/trajectory_0001" in r for r in rows)


def test_resume_reruns_when_camera_data_appears(dataset, tmp_path):
    from vio.scripts.run_midair_benchmark import run_benchmark
    out = tmp_path / "bench"
    run_benchmark(_args(dataset, out, "--resume"), make_plots=False)
    m1 = out / "trajectories" / "Kite_training_sunny_trajectory_0001" / "metrics.json"
    assert json.loads(m1.read_text())["methods"]["visual"] is None
    cache_h5 = next((out / "cache").rglob("sensor_records.hdf5"))
    with h5py.File(cache_h5, "r") as f:
        n = f["trajectory_0001/camera_data/color_left"].shape[0]
    _write_frames(dataset / "Kite_training" / "sunny" / "color_left" / "trajectory_0001" / "frames.zip", n)
    run_benchmark(_args(dataset, out, "--resume"), make_plots=False)
    assert json.loads(m1.read_text())["methods"]["visual"] is not None


def test_visual_failure_falls_back_without_crashing(dataset, tmp_path):
    """Random-texture frames give few valid rotations; the visual run must still complete."""
    r = discover(dataset, tmp_path / "cache", trajectories=["0"])
    vio_cfg = yaml.safe_load((REPO / "vio/configs/midair_vio.yaml").read_text())
    base = yaml.safe_load((REPO / "baseline/configs/midair_baseline.yaml").read_text())["midair"]
    out = run_trajectory(r.entries[0], base, vio_cfg, "left", CUT, [10])
    assert out["methods"]["visual"] is not None
    assert out["visual_statistics"]["frames_processed"] > 0


# ------------------------------------------------------------- no ground-truth leakage
def test_ground_truth_after_cutoff_does_not_change_estimates(tmp_path):
    vio_cfg = yaml.safe_load((REPO / "vio/configs/midair_vio.yaml").read_text())
    base = yaml.safe_load((REPO / "baseline/configs/midair_baseline.yaml").read_text())["midair"]
    est = []
    for i, offset in enumerate((0.0, 500.0)):
        root = make_dataset(tmp_path / f"d{i}" / "MidAir", gt_offset_after=offset)
        (e,) = discover(root, tmp_path / f"cache{i}", trajectories=["0"]).entries
        est.append(run_trajectory(e, base, vio_cfg, "left", CUT, [10], return_estimates=True)["_estimates"])
    np.testing.assert_array_equal(est[0]["imu_only"], est[1]["imu_only"])
    np.testing.assert_array_equal(est[0]["visual"], est[1]["visual"])
    # the oracle is not an estimator: it reads the true attitude, but not the true position
    np.testing.assert_array_equal(est[0]["ground_truth_attitude_oracle"], est[1]["ground_truth_attitude_oracle"])


def test_visual_subset_keeps_stored_full_results(dataset, tmp_path):
    """A later subset run with --resume must not throw away a stored visual result."""
    from vio.scripts.run_midair_benchmark import run_benchmark
    out = tmp_path / "bench"
    run_benchmark(_args(dataset, out, "--resume"), make_plots=False)
    m0 = out / "trajectories" / "Kite_training_sunny_trajectory_0000" / "metrics.json"
    stamp = m0.stat().st_mtime_ns
    s = run_benchmark(_args(dataset, out, "--resume", "--visual-subset", "sunny:5"), make_plots=False)
    assert m0.stat().st_mtime_ns == stamp
    assert json.loads(m0.read_text())["methods"]["visual"] is not None
    assert s["visual_subset"] == ["sunny:trajectory_0005"]
