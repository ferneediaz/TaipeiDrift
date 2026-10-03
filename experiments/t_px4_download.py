"""Select and download a diverse set of public PX4 multirotor logs.

Run from the repository root with:
    uv run --no-project --with pyulog --with pandas --with numpy python experiments/t_px4_download.py
    uv run --no-project --with pyulog --with pandas --with numpy python experiments/t_px4_download.py --additional 40 --rtk-search

The public dbinfo file is a large JSON array, so it is decoded incrementally.
"""
from __future__ import annotations

import argparse
import json
import re
import urllib.request
from collections import defaultdict
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RAW_DIR = ROOT / "data/raw/px4_logs"
INDEX_PATH = RAW_DIR / "dbinfo.json"
SELECTED_PATH = RAW_DIR / "selected_candidates.json"
MAX_TOTAL_BYTES = 3_000_000_000
CHUNK_BYTES = 1024 * 1024
MULTIROTOR = re.compile(r"quadrotor|multirotor|hexarotor|octo|tricopter", re.I)
SIMULATOR = re.compile(r"\b(?:HIL|SITL|simulation|simulator)\b", re.I)
VERSION = re.compile(r"^v?(\d+)\.(\d+)(?:\.(\d+))?")
RTK_HINT = re.compile(r"\b(?:RTK(?:LIB)?|F9P|moving[ _-]?baseline|differential(?: GPS)?|base[- ]rover)\b", re.I)


def iter_json_array(path: Path):
    decoder = json.JSONDecoder()
    with path.open(encoding="utf-8") as stream:
        buffer = ""
        eof = False
        started = False
        while True:
            buffer = buffer.lstrip()
            if not buffer:
                if eof:
                    return
                buffer = stream.read(CHUNK_BYTES)
                if not buffer:
                    eof = True
                continue
            if not started:
                if buffer[0] != "[":
                    raise ValueError(f"Expected a JSON array in {path}")
                buffer = buffer[1:]
                started = True
                continue
            if buffer[0] == "]":
                return
            if buffer[0] == ",":
                buffer = buffer[1:]
                continue
            try:
                record, end = decoder.raw_decode(buffer)
            except json.JSONDecodeError:
                if eof:
                    raise
                chunk = stream.read(CHUNK_BYTES)
                if not chunk:
                    eof = True
                buffer += chunk
                continue
            yield record
            buffer = buffer[end:]


def version_tuple(release: str) -> tuple[int, int, int] | None:
    match = VERSION.match(release or "")
    if not match:
        return None
    major, minor, patch = (int(part or 0) for part in match.groups())
    if major < 1 or (major == 1 and minor < 12):
        return None
    return major, minor, patch


def labels_for(record: dict) -> list[str]:
    value = record.get("error_labels")
    if isinstance(value, list):
        return [str(label) for label in value]
    if isinstance(value, str) and value:
        return [value]
    return []


def is_candidate(record: dict) -> bool:
    if not isinstance(record, dict) or not record.get("log_id"):
        return False
    duration = float(record.get("duration_s") or 0)
    if not 300 <= duration <= 86_400:
        return False
    mav_type = str(record.get("mav_type") or "")
    airframe = str(record.get("airframe_type") or "")
    if not MULTIROTOR.search(mav_type + " " + airframe):
        return False
    if version_tuple(str(record.get("ver_sw_release") or "")) is None:
        return False
    rating = str(record.get("rating") or "").strip().lower()
    if "crash" in rating:
        return False
    labels = " ".join(labels_for(record))
    if "crash" in labels.lower():
        return False
    identifying_text = " ".join(
        str(record.get(key) or "")
        for key in ("airframe_name", "sys_hw", "description", "source", "vehicle_name")
    )
    if SIMULATOR.search(identifying_text):
        return False
    return True


def rank(record: dict, prioritize_rtk: bool = False) -> tuple:
    rating = str(record.get("rating") or "").strip().lower()
    quality = 0 if rating in {"good", "great", "bravo!", "bravo"} else 1 if not rating else 2
    version = version_tuple(str(record.get("ver_sw_release") or "")) or (0, 0, 0)
    try:
        log_date = date.fromisoformat(str(record.get("log_date") or "")).toordinal()
    except ValueError:
        log_date = 0
    return (
        -rtk_hint_score(record) if prioritize_rtk else 0,
        quality,
        len(labels_for(record)),
        -float(record.get("duration_s") or 0),
        tuple(-part for part in version),
        -log_date,
    )


def airframe_key(record: dict) -> str:
    return str(record.get("airframe_name") or record.get("airframe_type") or record.get("mav_type") or "unknown")


def rtk_hint_score(record: dict) -> int:
    text = " ".join(
        str(record.get(key) or "")
        for key in ("description", "feedback", "source", "sys_hw", "airframe_name", "airframe_type", "vehicle_name")
    )
    if re.search(r"\b(?:without|no|not)\s+RTK\b|\bRTK\s+(?:not available|unavailable|disabled)\b", text, re.I):
        return 0
    if not RTK_HINT.search(text):
        return 0
    specific = r"\b(?:with\s+RTK|RTK\s+(?:test|GPS|receiver)|F9P|survey\s+test|rover)\b"
    return 2 if re.search(specific, text, re.I) else 1


def has_rtk_hint(record: dict) -> bool:
    return rtk_hint_score(record) > 0


def choose(records: list[dict], limit: int, prioritize_rtk: bool = False) -> list[dict]:
    groups: dict[str, list[dict]] = defaultdict(list)
    for record in records:
        groups[airframe_key(record)].append(record)
    for group in groups.values():
        group.sort(key=lambda record: rank(record, prioritize_rtk))
    # Round-robin among common airframes; RTK search elevates explicit receiver hints.
    if prioritize_rtk:
        keys = sorted(groups, key=lambda key: (-sum(has_rtk_hint(record) for record in groups[key]), -len(groups[key]), key))[:20]
    else:
        keys = sorted(groups, key=lambda key: (-len(groups[key]), key))[:20]
    chosen: list[dict] = []
    position = 0
    while len(chosen) < limit:
        added = False
        for key in keys:
            if position < len(groups[key]):
                chosen.append(groups[key][position])
                added = True
                if len(chosen) >= limit:
                    break
        if not added:
            break
        position += 1
    return chosen


def load_selected() -> dict[str, dict]:
    if not SELECTED_PATH.exists():
        return {}
    data = json.loads(SELECTED_PATH.read_text(encoding="utf-8"))
    rows = data.get("logs", []) if isinstance(data, dict) else data
    return {row["log_id"]: row for row in rows if row.get("log_id")}


def download_one(record: dict, destination: Path, remaining_bytes: int) -> int:
    log_id = record["log_id"]
    url = f"https://review.px4.io/download?log={log_id}"
    request = urllib.request.Request(url, method="HEAD", headers={"User-Agent": "TaipeiDrift research dataset fetcher"})
    with urllib.request.urlopen(request, timeout=90) as response:
        length_header = response.headers.get("Content-Length")
    if length_header is None:
        raise RuntimeError("server did not provide Content-Length; refusing an unbounded download")
    expected = int(length_header)
    if expected <= 0:
        raise RuntimeError(f"invalid Content-Length: {expected}")
    if expected > remaining_bytes:
        raise OverflowError(f"next log is {expected} bytes; only {remaining_bytes} bytes remain")

    request = urllib.request.Request(url, headers={"User-Agent": "TaipeiDrift research dataset fetcher"})
    temporary = destination.with_suffix(destination.suffix + ".part")
    written = 0
    try:
        with urllib.request.urlopen(request, timeout=180) as response, temporary.open("wb") as output:
            while chunk := response.read(CHUNK_BYTES):
                written += len(chunk)
                if written > remaining_bytes:
                    raise OverflowError("response exceeded the remaining download budget")
                output.write(chunk)
        if written != expected:
            raise IOError(f"Content-Length was {expected}, received {written}")
        temporary.replace(destination)
        return written
    except Exception:
        temporary.unlink(missing_ok=True)
        raise


def save_selected(records: dict[str, dict]) -> None:
    SELECTED_PATH.write_text(
        json.dumps({"logs": list(records.values())}, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--limit", type=int, default=60, help="initial candidate cap (default: 60)")
    parser.add_argument("--additional", type=int, default=0, help="download this many more logs, e.g. 40 for RTK search")
    parser.add_argument("--rtk-search", action="store_true", help="prefer RTK/GNSS hardware hints in dbinfo")
    args = parser.parse_args()
    if args.limit < 0 or args.additional < 0:
        parser.error("candidate limits must be non-negative")
    if not INDEX_PATH.exists():
        raise FileNotFoundError(f"Fetch {INDEX_PATH} from https://review.px4.io/dbinfo first")

    index_count = 0
    eligible: list[dict] = []
    for record in iter_json_array(INDEX_PATH):
        index_count += 1
        if is_candidate(record):
            eligible.append(record)
    eligible_by_id = {record["log_id"]: record for record in eligible}

    selected = load_selected()
    existing_files = {path.stem: path for path in RAW_DIR.glob("*.ulg")}
    for log_id in existing_files:
        if log_id in eligible_by_id:
            selected.setdefault(log_id, eligible_by_id[log_id])

    total_bytes = sum(path.stat().st_size for path in existing_files.values())
    if total_bytes > MAX_TOTAL_BYTES:
        raise RuntimeError(f"existing .ulg downloads exceed the {MAX_TOTAL_BYTES}-byte limit")

    new_target = args.additional if args.additional else max(0, args.limit - len(existing_files))
    already = set(existing_files)
    available = [record for record in eligible if record["log_id"] not in already]
    candidates = choose(available, new_target, prioritize_rtk=args.rtk_search)
    downloaded_this_run = 0
    failures = 0
    size_limit_skips = 0

    for record in candidates:
        destination = RAW_DIR / f"{record['log_id']}.ulg"
        try:
            size = download_one(record, destination, MAX_TOTAL_BYTES - total_bytes)
        except OverflowError:
            size_limit_skips += 1
            continue
        except Exception:
            failures += 1
            break
        total_bytes += size
        downloaded_this_run += 1
        selected[record["log_id"]] = record
        save_selected(selected)

    downloaded_rows = [selected[log_id] for log_id in sorted(existing_files | {path.stem: path for path in RAW_DIR.glob("*.ulg")}) if log_id in selected]
    summary = [
        {
            "log_id": row.get("log_id"),
            "duration_s": row.get("duration_s"),
            "airframe": row.get("airframe_name") or row.get("airframe_type") or row.get("mav_type"),
            "hardware": row.get("sys_hw"),
            "version": row.get("ver_sw_release"),
            "rating": row.get("rating"),
        }
        for row in downloaded_rows[:5]
    ]
    print(json.dumps({
        "logs_listed": index_count,
        "eligible_multirotor_candidates": len(eligible),
        "downloaded_total": len(existing_files | {path.stem: path for path in RAW_DIR.glob("*.ulg")}),
        "downloaded_this_run": downloaded_this_run,
        "total_download_bytes": total_bytes,
        "max_download_bytes": MAX_TOTAL_BYTES,
        "failed_downloads": failures,
        "skipped_candidates_over_budget": size_limit_skips,
        "five_row_summary": summary,
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
