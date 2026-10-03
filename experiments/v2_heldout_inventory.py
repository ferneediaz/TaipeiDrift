#!/usr/bin/env python3
"""Inventory the downloaded MUN-FRL and OrthoLoC held-out samples.

Run from the repository root:
  .venv/bin/python experiments/v2_heldout_inventory.py
"""

from __future__ import annotations

import argparse
import datetime as dt
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MUN_BAG = ROOT / "data/raw/mun_frl/lighthouse_francis_sample.bag"
DEFAULT_MUN_PPK = ROOT / "data/raw/mun_frl/flight_dataset5_ppk.pos"
DEFAULT_ORTHOLOC = ROOT / "data/raw/ortholoc/test_outPlace_L08_xDOP"


def rate_hz(timestamps_ns: list[int]) -> float | None:
    if len(timestamps_ns) < 2:
        return None
    span_s = (max(timestamps_ns) - min(timestamps_ns)) / 1e9
    return (len(timestamps_ns) - 1) / span_s if span_s > 0 else None


def format_rate(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.2f} Hz"


def inspect_ortholoc(directory: Path) -> None:
    files = sorted(directory.glob("L08_xDOP*.npz"))
    if not files:
        raise FileNotFoundError(f"No OrthoLoC samples found in {directory}")

    sample_ids: list[str] = []
    shapes: dict[str, set[tuple[int, ...]]] = defaultdict(set)
    field_sets: set[tuple[str, ...]] = set()
    required = {"image_query", "image_dop", "dsm", "point_map", "extrinsics", "intrinsics"}
    for path in files:
        with np.load(path, allow_pickle=False) as sample:
            fields = tuple(sorted(sample.files))
            field_sets.add(fields)
            if not required.issubset(fields):
                raise ValueError(f"{path}: missing query/map/pose fields")
            sample_ids.append(str(sample["sample_id"].item()))
            for key in required:
                shapes[key].add(tuple(sample[key].shape))

    if len(field_sets) != 1:
        raise ValueError("OrthoLoC sample files do not share one schema")
    print("OrthoLoC test_outPlace / L08_xDOP (external DOP):")
    print(f"  files / real query frames: {len(files)}")
    print(f"  sample IDs: {sample_ids[0]} .. {sample_ids[-1]}")
    print("  timestamp/rate: unavailable (no per-frame time in the released NPZ samples)")
    print(f"  query image: {sorted(shapes['image_query'])}")
    print(f"  map DOP: {sorted(shapes['image_dop'])}; DSM: {sorted(shapes['dsm'])}")
    print(f"  depth/point map: {sorted(shapes['point_map'])}")
    print(f"  pose/calibration: extrinsics {sorted(shapes['extrinsics'])}; intrinsics {sorted(shapes['intrinsics'])}")
    print(f"  all NPZ fields: {', '.join(next(iter(field_sets)))}")
    print("  raw IMU/barometer/GNSS streams: absent from these NPZ samples")
    print("  coordinates: scene-local/sanitized; no global lat/lon fields")


def image_dimensions(reader: AnyReader, connection, rawdata: bytes) -> tuple[int, ...] | None:
    message = reader.deserialize(rawdata, connection.msgtype)
    if hasattr(message, "height") and hasattr(message, "width"):
        return (int(message.height), int(message.width))
    if hasattr(message, "data"):
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
        if image is not None:
            return tuple(int(size) for size in image.shape)
    return None


def inspect_mun_bag(path: Path) -> None:
    if not path.is_file():
        raise FileNotFoundError(path)

    counts: Counter[str] = Counter()
    timestamps: dict[str, list[int]] = defaultdict(list)
    first_images: dict[str, tuple[AnyReader, object, bytes]] = {}
    with AnyReader([path]) as reader:
        connection_count = len(reader.connections)
        topic_types = {connection.topic: connection.msgtype for connection in reader.connections}
        for connection, timestamp_ns, rawdata in reader.messages():
            topic = connection.topic
            counts[topic] += 1
            timestamps[topic].append(timestamp_ns)
            if (
                connection.msgtype in ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage")
                and topic not in first_images
            ):
                first_images[topic] = (reader, connection, rawdata)

        all_stamps = [stamp for topic_stamps in timestamps.values() for stamp in topic_stamps]
        duration_s = (max(all_stamps) - min(all_stamps)) / 1e9 if all_stamps else 0.0
        print("MUN-FRL Lighthouse sample ROS bag:")
        print(f"  file bytes: {path.stat().st_size}")
        print(f"  bag duration: {duration_s:.3f} s; indexed topics: {connection_count}")
        print("  topic inventory (count; mean recorder-time rate over topic span):")
        for topic in sorted(counts):
            dims = ""
            if topic_types.get(topic) in ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"):
                first = first_images.get(topic)
                shape = image_dimensions(first[0], first[1], first[2]) if first else None
                dims = f"; first image shape={shape}" if shape else ""
            print(f"    {topic}: {counts[topic]}; {format_rate(rate_hz(timestamps[topic]))}{dims}")

        if "/fix" in counts:
            fix_fields = ("latitude", "longitude", "altitude", "status.status", "position_covariance")
            print(f"  raw RTK topic: /fix; fields: {', '.join(fix_fields)}")
        else:
            print("  raw RTK topic /fix: absent")
        for topic in ("/fix_ppk", "/ins_ppk", "/fix_frl"):
            print(f"  {topic}: {'present' if topic in counts else 'absent'}")
        baro_topics = [topic for topic in counts if "baro" in topic.lower() or "pressure" in topic.lower()]
        print(f"  barometer topics: {', '.join(sorted(baro_topics)) if baro_topics else 'absent'}")
        image_topics = [
            topic for topic in counts
            if topic_types.get(topic) in ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage")
        ]
        print("  image frame counts: " + ", ".join(f"{topic}={counts[topic]}" for topic in sorted(image_topics)))
        imu_topics = [topic for topic in counts if topic == "/imu/data"]
        print(
            f"  IMU topics: {', '.join(imu_topics) if imu_topics else 'absent'}; "
            "raw fields: angular_velocity, linear_acceleration, orientation"
        )
        print("  estimator output topics are inventoried, not treated as independent ground truth")


def inspect_ppk(path: Path) -> None:
    if not path.is_file():
        raise FileNotFoundError(path)
    timestamps: list[dt.datetime] = []
    quality_counts: Counter[str] = Counter()
    fields = ""
    with path.open(encoding="ascii", errors="replace") as stream:
        for line in stream:
            stripped = line.strip()
            if stripped.startswith("%") and "GPST" in stripped and "latitude" in stripped:
                fields = stripped.lstrip("% ")
            elif stripped and not stripped.startswith("%"):
                columns = stripped.split()
                if len(columns) >= 6:
                    timestamps.append(dt.datetime.strptime(f"{columns[0]} {columns[1]}", "%Y/%m/%d %H:%M:%S.%f"))
                    quality_counts[columns[5]] += 1
    if not timestamps:
        raise ValueError(f"No PPK solution records found in {path}")

    intervals = np.asarray([(b - a).total_seconds() for a, b in zip(timestamps, timestamps[1:])])
    median_dt = float(np.median(intervals)) if len(intervals) else 0.0
    observed_rate = 1.0 / median_dt if median_dt > 0 else None
    print("MUN-FRL Lighthouse PPK position file:")
    print(f"  file bytes: {path.stat().st_size}; solution rows: {len(timestamps)}")
    print(
        f"  GPST span: {timestamps[0].isoformat()} .. {timestamps[-1].isoformat()}; "
        f"rate: {format_rate(observed_rate)}"
    )
    print(f"  fields: {fields}")
    print(f"  solution quality counts (Q=1 fixed): {dict(sorted(quality_counts.items()))}")
    print("  truth fields: latitude, longitude, WGS84 ellipsoidal height, Q, satellite count, position uncertainties, age, ratio")
    print("  orientation: not present in this standalone .pos file")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mun-bag", type=Path, default=DEFAULT_MUN_BAG)
    parser.add_argument("--mun-ppk", type=Path, default=DEFAULT_MUN_PPK)
    parser.add_argument("--ortholoc-dir", type=Path, default=DEFAULT_ORTHOLOC)
    args = parser.parse_args()

    inspect_mun_bag(args.mun_bag)
    inspect_ppk(args.mun_ppk)
    inspect_ortholoc(args.ortholoc_dir)


if __name__ == "__main__":
    main()
