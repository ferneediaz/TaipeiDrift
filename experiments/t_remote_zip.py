"""Read a remote ZIP over HTTP range requests: list members, extract selected ones.

Track C (datasets/replay). No full download needed when the server honours
`Range:` (checked: download.ifi.uzh.ch AGZ.zip, Dropbox per-file dl=1 links).
Nearby members are fetched with one range request per run (<= 128 MB), so extracting
a contiguous window of images costs about its compressed size.

Usage:
  .venv/bin/python experiments/t_remote_zip.py list URL [--grep REGEX] [--out members.csv]
  .venv/bin/python experiments/t_remote_zip.py get URL --dest DIR (--grep REGEX | --names FILE) [--every N] [--limit N]
"""

from __future__ import annotations

import argparse
import csv
import re
import struct
import sys
import time
import urllib.request
import zlib
from dataclasses import dataclass
from pathlib import Path


def fetch(url: str, start: int, end: int, tries: int = 6) -> bytes:
    """Inclusive byte range [start, end]."""
    for k in range(tries):
        try:
            req = urllib.request.Request(url, headers={"Range": f"bytes={start}-{end}"})
            with urllib.request.urlopen(req, timeout=300) as r:
                if r.status != 206:
                    raise RuntimeError(f"server ignored Range (status {r.status})")
                data = r.read()
            if len(data) != end - start + 1:
                raise RuntimeError(f"short read {len(data)} != {end - start + 1}")
            return data
        except Exception as e:  # network flakiness: retry with backoff
            if k == tries - 1:
                raise
            print(f"retry {k + 1}: {e}", file=sys.stderr)
            time.sleep(2 * (k + 1))
    raise AssertionError


def total_size(url: str) -> int:
    req = urllib.request.Request(url, headers={"Range": "bytes=0-0"})
    with urllib.request.urlopen(req, timeout=60) as r:
        cr = r.headers["Content-Range"]
    return int(cr.split("/")[1])


@dataclass
class Member:
    name: str
    method: int
    csize: int
    usize: int
    crc: int
    offset: int  # local header offset


def central_directory(url: str) -> tuple[int, list[Member]]:
    size = total_size(url)
    tail_len = min(size, 65536 + 22 + 20 + 56)
    tail = fetch(url, size - tail_len, size - 1)
    i = tail.rfind(b"PK\x05\x06")
    if i < 0:
        raise RuntimeError("no end-of-central-directory record")
    _, _, _, _, n_entries, cd_size, cd_off, _ = struct.unpack("<IHHHHIIH", tail[i : i + 22])
    j = tail.rfind(b"PK\x06\x07", 0, i)  # zip64 locator
    if j >= 0:
        (z64_off,) = struct.unpack("<Q", tail[j + 8 : j + 16])
        rec = fetch(url, z64_off, z64_off + 55)
        assert rec[:4] == b"PK\x06\x06"
        n_entries, cd_size, cd_off = struct.unpack("<QQQ", rec[32:56])
    cd = fetch(url, cd_off, cd_off + cd_size - 1)
    out: list[Member] = []
    p = 0
    while p < len(cd) and cd[p : p + 4] == b"PK\x01\x02":
        (method, _t, _d, crc, csize, usize, nlen, xlen, clen) = struct.unpack("<HHHIIIHHH", cd[p + 10 : p + 34])
        (offset,) = struct.unpack("<I", cd[p + 42 : p + 46])
        name = cd[p + 46 : p + 46 + nlen].decode("utf-8", "replace")
        extra = cd[p + 46 + nlen : p + 46 + nlen + xlen]
        q = 0
        while q + 4 <= len(extra):  # zip64 extended info
            hid, hlen = struct.unpack("<HH", extra[q : q + 4])
            if hid == 1:
                vals = extra[q + 4 : q + 4 + hlen]
                k = 0
                if usize == 0xFFFFFFFF:
                    (usize,) = struct.unpack("<Q", vals[k : k + 8]); k += 8
                if csize == 0xFFFFFFFF:
                    (csize,) = struct.unpack("<Q", vals[k : k + 8]); k += 8
                if offset == 0xFFFFFFFF:
                    (offset,) = struct.unpack("<Q", vals[k : k + 8]); k += 8
            q += 4 + hlen
        out.append(Member(name, method, csize, usize, crc, offset))
        p += 46 + nlen + xlen + clen
    if len(out) != n_entries:
        print(f"warning: parsed {len(out)} of {n_entries} entries", file=sys.stderr)
    return size, out


def _decode(m: Member, blob: bytes, base: int) -> bytes:
    """Decode member `m` from `blob`, which holds archive bytes starting at offset `base`."""
    h = m.offset - base
    if blob[h : h + 4] != b"PK\x03\x04":
        raise RuntimeError(f"no local header for {m.name}")
    nlen, xlen = struct.unpack("<HH", blob[h + 26 : h + 30])
    s = h + 30 + nlen + xlen
    raw = blob[s : s + m.csize]
    if len(raw) != m.csize:
        raise RuntimeError(f"truncated blob for {m.name}")
    if m.method == 0:
        data = raw
    elif m.method == 8:
        data = zlib.decompress(raw, -15)
    else:
        raise RuntimeError(f"unsupported method {m.method} for {m.name}")
    if zlib.crc32(data) != m.crc:
        raise RuntimeError(f"CRC mismatch for {m.name}")
    return data


def extract_batched(url: str, size: int, members: list[Member],
                    max_span: int = 128 << 20, max_gap: int = 4 << 20):
    """Yield (member, data); one range request per run of nearby members.
    The local header's extra field is not in the central directory: allow 64 KiB slack."""
    slack = 30 + 65536
    ms = sorted(members, key=lambda m: m.offset)
    i = 0
    while i < len(ms):
        start = ms[i].offset
        end = ms[i].offset + slack + len(ms[i].name.encode()) + ms[i].csize
        j = i
        while j + 1 < len(ms):
            n = ms[j + 1]
            n_end = n.offset + slack + len(n.name.encode()) + n.csize
            if n.offset - end > max_gap or n_end - start > max_span:
                break
            end = max(end, n_end)
            j += 1
        blob = fetch(url, start, min(end, size) - 1)
        for m in ms[i : j + 1]:
            yield m, _decode(m, blob, start)
        i = j + 1


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["list", "get"])
    ap.add_argument("url")
    ap.add_argument("--grep")
    ap.add_argument("--names", help="text file with one member name per line")
    ap.add_argument("--every", type=int, default=1, help="keep every N-th matching member (sorted by name)")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--out")
    ap.add_argument("--dest")
    a = ap.parse_args()

    size, members = central_directory(a.url)
    sel = members
    if a.grep:
        rx = re.compile(a.grep)
        sel = [m for m in sel if rx.search(m.name)]
    if a.names:
        wanted = set(Path(a.names).read_text().split("\n")) - {""}
        sel = [m for m in sel if m.name in wanted]
    sel = sorted(sel, key=lambda m: m.name)[:: a.every]
    if a.limit:
        sel = sel[: a.limit]

    if a.cmd == "list":
        print(f"archive {size} B, {len(members)} members, {len(sel)} selected, "
              f"{sum(m.csize for m in sel)} B compressed")
        if a.out:
            with open(a.out, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["name", "method", "csize", "usize", "crc", "offset"])
                for m in sel:
                    w.writerow([m.name, m.method, m.csize, m.usize, m.crc, m.offset])
        return

    dest = Path(a.dest)
    todo = [m for m in sel if not m.name.endswith("/")
            and not ((dest / m.name).exists() and (dest / m.name).stat().st_size == m.usize)]
    done = 0
    for m, data in extract_batched(a.url, size, todo):
        p = dest / m.name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(data)
        done += 1
        if done % 500 == 0:
            print(f"{done}/{len(todo)} extracted", file=sys.stderr)
    print(f"extracted {done} new members into {dest} ({len(sel) - len(todo)} already present)")


if __name__ == "__main__":
    main()
