"""M0 data inspection: header info and strip-wise statistics for the raw rasters (read-only)."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import rasterio
from rasterio.windows import Window

ROWS_PER_STRIP = 1024


def raster_stats(path: Path) -> dict:
    """Compute global statistics with strip-wise windowed reads (never the whole raster)."""
    with rasterio.open(path) as ds:
        nd = ds.nodata
        n_total = ds.width * ds.height
        n_nodata = n_zero = n_neg = n_valid = 0
        vmin, vmax, vsum = np.inf, -np.inf, 0.0
        hist_edges = np.arange(-200, 4201, 100)
        hist = np.zeros(len(hist_edges) - 1, dtype=np.int64)
        spike_cnt = 0
        for r0 in range(0, ds.height, ROWS_PER_STRIP):
            h = min(ROWS_PER_STRIP, ds.height - r0)
            a = ds.read(1, window=Window(0, r0, ds.width, h)).astype(np.float64)
            nodata = a == nd
            v = a[~nodata]
            n_nodata += int(nodata.sum())
            n_valid += v.size
            if v.size:
                vmin, vmax = min(vmin, v.min()), max(vmax, v.max())
                vsum += v.sum()
                n_zero += int((v == 0).sum())
                n_neg += int((v < 0).sum())
                hist += np.histogram(v, hist_edges)[0]
            # crude spike detector: |z - mean of 4-neighbours| > 300 m, both cells valid
            z = np.where(nodata, np.nan, a)
            nb = np.nanmean(np.stack([z[1:-1, :-2], z[1:-1, 2:], z[:-2, 1:-1], z[2:, 1:-1]]), axis=0) \
                if z.shape[0] > 2 else None
            if nb is not None:
                spike_cnt += int(np.nansum(np.abs(z[1:-1, 1:-1] - nb) > 300))
        return dict(
            file=str(path), width=ds.width, height=ds.height, dtype=ds.dtypes[0], nodata=nd,
            pixel=(ds.transform.a, ds.transform.e), bounds=tuple(ds.bounds), crs_wkt_name=ds.crs.to_wkt()[:40],
            epsg_guess=ds.crs.to_epsg(), area_or_point=ds.tags().get("AREA_OR_POINT"),
            mb_float32=n_total * 4 / 1e6, frac_nodata=n_nodata / n_total, n_valid=n_valid,
            n_zero=n_zero, n_negative=n_neg, min=float(vmin), max=float(vmax), mean=vsum / n_valid,
            spikes_gt300m=spike_cnt,
            hist={f"{int(hist_edges[i])}": int(hist[i]) for i in range(len(hist)) if hist[i]},
        )


if __name__ == "__main__":
    out = [raster_stats(Path(p)) for p in sys.argv[1:]]
    print(json.dumps(out, indent=1))
