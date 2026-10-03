"""Build monthly CWA sunshine normals and geometric possible-sunshine hours.

The CWA sunshine and cloud tables are saved verbatim under data/raw/cwa_sunshine/;
this script parses those local files and writes data/processed/taiwan_sunshine/.

Possible sunshine uses the Cooper declination approximation for a common year:
    delta = 23.45 deg * sin(2*pi*(284 + day_of_year)/365)
At latitude phi, sunrise/sunset hour angle H0 = acos(-tan(phi)*tan(delta));
day length = 24*H0/pi hours (geometric solar-center altitude 0 deg). Monthly
possible hours sum the daily lengths for each month of the representative year
2021. No API key or network request is required to run this script.
"""
from __future__ import annotations

import calendar
import csv
import math
from datetime import date
from html.parser import HTMLParser
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RAW_DIR = ROOT / "data" / "raw" / "cwa_sunshine"
OUTPUT = ROOT / "data" / "processed" / "taiwan_sunshine" / "monthly_sunshine.csv"
REPRESENTATIVE_YEAR = 2021

SUNSHINE_URL = "https://www.cwa.gov.tw/V8/C/C/Statistics/MonthlyMean/MOD/Taiwan_sunshine.html"
CLOUD_URL = "https://www.cwa.gov.tw/V8/C/C/Statistics/MonthlyMean/MOD/Taiwan_CamtMean.html"
STATION_URL = "https://hdps.cwa.gov.tw/static/state.html"
SOURCE_URL = f"{SUNSHINE_URL}; {CLOUD_URL}; {STATION_URL}"

# Source station labels are checked against the CWA tables and station registry.
STATIONS = (
    ("Taipei", "466920", "臺北"),
    ("Taichung", "467490", "臺中"),
    ("Kaohsiung", "467440", "高雄"),
    ("Hualien", "466990", "花蓮"),
    ("Tainan", "467410", "臺南"),
    ("Hengchun", "467590", "恆春"),
)


class RowParser(HTMLParser):
    """Collect HTML table rows as (header, text) cells, including bare fragment tables."""

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.rows: list[list[tuple[str | None, str]]] = []
        self.row: list[tuple[str | None, str]] | None = None
        self.cell_header: str | None = None
        self.cell_parts: list[str] | None = None

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        if tag == "tr":
            self.row = []
        elif tag in ("th", "td") and self.row is not None:
            self.cell_header = dict(attrs).get("headers")
            self.cell_parts = []

    def handle_data(self, data: str) -> None:
        if self.cell_parts is not None:
            self.cell_parts.append(data)

    def handle_endtag(self, tag: str) -> None:
        if tag in ("th", "td") and self.row is not None and self.cell_parts is not None:
            self.row.append((self.cell_header, " ".join(self.cell_parts).strip()))
            self.cell_header = None
            self.cell_parts = None
        elif tag == "tr" and self.row is not None:
            if self.row:
                self.rows.append(self.row)
            self.row = None


def parse_table(path: Path) -> list[list[tuple[str | None, str]]]:
    parser = RowParser()
    parser.feed(path.read_text(encoding="utf-8"))
    return parser.rows


def parse_month_value(value: str) -> float:
    """Parse a table value, representing blank or explicit missing markers as NaN."""
    cleaned = value.strip()
    if cleaned in {"", "-", "--", "－", "—", "NA", "N/A"}:
        return math.nan
    return float(cleaned)


def parse_monthly_table(path: Path) -> dict[str, dict[str, object]]:
    records: dict[str, dict[str, object]] = {}
    for cells in parse_table(path):
        values = {header: text for header, text in cells if header is not None}
        station = values.get("loc", "").strip()
        if not station:
            continue
        months = [parse_month_value(values.get(f"m{i}", "")) for i in range(12)]
        period = values.get("period", "").replace("~", "-")
        records[station] = {"months": months, "period": period}
    return records


def parse_station_registry(path: Path) -> dict[str, tuple[float, float, str]]:
    """Return station ID -> (latitude, longitude, Chinese name) from CWA registry tables."""
    targets = {station_id for _, station_id, _ in STATIONS}
    found: dict[str, tuple[float, float, str]] = {}
    for cells in parse_table(path):
        values = [text for _, text in cells]
        if len(values) < 6 or values[0] not in targets:
            continue
        station_id, name = values[0], values[1]
        # CWA registry columns: ID, name, type, elevation, longitude, latitude.
        found[station_id] = (float(values[5]), float(values[4]), name)
    missing = targets - found.keys()
    if missing:
        raise ValueError(f"CWA station registry is missing target IDs: {', '.join(sorted(missing))}")
    return found


def daily_possible_hours(latitude_deg: float, day_of_year: int) -> float:
    """Geometric sunrise-to-sunset duration with solar-center elevation set to 0 degrees."""
    latitude = math.radians(latitude_deg)
    declination = math.radians(23.45) * math.sin(2 * math.pi * (284 + day_of_year) / 365)
    cos_hour_angle = -math.tan(latitude) * math.tan(declination)
    if cos_hour_angle <= -1:
        return 24.0
    if cos_hour_angle >= 1:
        return 0.0
    hour_angle = math.acos(cos_hour_angle)
    return 24.0 * hour_angle / math.pi


def monthly_possible_hours(latitude_deg: float, month: int) -> float:
    """Sum the geometric daily daylight duration through the month in representative year 2021."""
    days = calendar.monthrange(REPRESENTATIVE_YEAR, month)[1]
    return sum(
        daily_possible_hours(latitude_deg, date(REPRESENTATIVE_YEAR, month, day).timetuple().tm_yday)
        for day in range(1, days + 1)
    )


def main() -> None:
    sunshine = parse_monthly_table(RAW_DIR / "Taiwan_sunshine.html")
    clouds = parse_monthly_table(RAW_DIR / "Taiwan_CamtMean.html")
    coordinates = parse_station_registry(RAW_DIR / "station_registry.html")
    rows: list[dict[str, object]] = []

    for station, station_id, source_name in STATIONS:
        if source_name not in sunshine:
            raise ValueError(f"CWA sunshine table is missing station {source_name} ({station_id})")
        sunshine_record = sunshine[source_name]
        cloud_record = clouds.get(source_name)
        if not sunshine_record["period"].startswith("1991-2020"):
            raise ValueError(f"Unexpected sunshine normal period for {station}: {sunshine_record['period']}")
        if cloud_record is not None and cloud_record["period"] != sunshine_record["period"]:
            raise ValueError(f"CWA sunshine/cloud periods differ for {station}")
        lat, lon, registry_name = coordinates[station_id]
        if registry_name != source_name:
            raise ValueError(f"Station ID {station_id} resolves to {registry_name}, expected {source_name}")
        sunlight = sunshine_record["months"]
        cloud = cloud_record["months"] if cloud_record is not None else [math.nan] * 12
        assert isinstance(sunlight, list) and isinstance(cloud, list)
        if any(not math.isfinite(value) for value in sunlight):
            raise ValueError(f"CWA sunshine table has missing monthly values for {station}")
        for month, sunshine_hours in enumerate(sunlight, start=1):
            possible_hours = monthly_possible_hours(lat, month)
            rows.append({
                "station": station,
                "station_id": station_id,
                "lat": lat,
                "lon": lon,
                "month": month,
                "sunshine_hours": sunshine_hours,
                "possible_hours": possible_hours,
                "sunshine_fraction": sunshine_hours / possible_hours,
                "cloud_tenths": cloud[month - 1],
                "source_url": SOURCE_URL,
            })

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "station", "station_id", "lat", "lon", "month", "sunshine_hours",
        "possible_hours", "sunshine_fraction", "cloud_tenths", "source_url",
    ]
    with OUTPUT.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({
                **row,
                "lat": f"{row['lat']:.6f}",
                "lon": f"{row['lon']:.6f}",
                "sunshine_hours": f"{row['sunshine_hours']:.1f}",
                "possible_hours": f"{row['possible_hours']:.3f}",
                "sunshine_fraction": f"{row['sunshine_fraction']:.6f}",
                "cloud_tenths": f"{row['cloud_tenths']:.1f}",
            })

    months = [f"{month:02d}" for month in range(1, 13)]
    print("Monthly sunshine fraction (sunshine hours / geometric possible hours)")
    print(f"{'Station':<11}" + " ".join(f"{month:>5}" for month in months))
    for station, _, _ in STATIONS:
        station_rows = [row for row in rows if row["station"] == station]
        print(f"{station:<11}" + " ".join(f"{row['sunshine_fraction']:5.2f}" for row in station_rows))
    print("\nAnnual mean sunshine fraction (annual sunshine hours / annual possible hours)")
    for station, _, _ in STATIONS:
        station_rows = [row for row in rows if row["station"] == station]
        annual = sum(float(row["sunshine_hours"]) for row in station_rows) / sum(
            float(row["possible_hours"]) for row in station_rows
        )
        print(f"{station:<11}{annual:.3f}")
    print(f"\nWrote {len(rows)} rows to {OUTPUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
