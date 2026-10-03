"""Pure run-logging policy helpers."""
import math


def gps_csv_values(available, fix, now_s, max_age_s=2.5):
    """Return raw lat/lon/alt only while the GNSS gate is open and the fix is fresh."""
    if available and fix is not None and 0 <= now_s - fix[0] < max_age_s:
        return [fix[1], fix[2], fix[3]]
    return [math.nan, math.nan, math.nan]
