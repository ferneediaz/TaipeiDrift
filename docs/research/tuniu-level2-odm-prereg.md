# Pre-registration addendum: Level 2 with the OpenDroneMap survey products

> **Status (2026-10-03 19:55): NOT RUN.** The team dropped the survey-flight 3D map from the project scope
> (the realistic setup, existing orthophoto + Copernicus terrain, already passes). No run with these products
> was started, so there is no unreported result.

Written 2026-10-03 ~19:50 Taipei, before any run with these products. Same estimator, fix rule, gates,
seeds (0–19), metrics and pass criteria as [`tuniu-level2-prereg.md`](tuniu-level2-prereg.md); only the map
and/or the terrain change. Purpose: measure what a survey-flight 3D map adds over the realistic setup
(existing orthophoto + Copernicus terrain), which already passed.

## Products (made by OpenDroneMap 3.6.2 from the 2019-09-16 survey flight, 297 photos, RTK)

- Orthophoto `data/processed/x_tuniu_survey_odm/odm_orthophoto/odm_orthophoto.tif` (5 cm, UTM 51N),
  reprojected by `x5_tuniu_closed_loop.py prepare-map` to `data/processed/x_tuniu_l2/maps/odm_0.5m.tif`
  (EPSG:3826, 0.5 m/px). Format conversion only.
- Surface model `data/processed/x_tuniu_survey_odm/odm_dem/dsm.tif` (10 cm, ellipsoidal heights, trees and
  buildings included); `--terrain-dz 0`; cells without data fall back to Copernicus (built into `Terrain`).

## Configurations (closed loop, heading `dji`, ground `dem_lifted`, 20 seeds)

| Tag | Map | Map offset | Terrain |
|---|---|---|---|
| (reference, already run) | OAM 2019-12 `main` | step 1 | Copernicus |
| `main_dsm` | OAM 2019-12 `main` | step 1 | ODM DSM |
| `odm_cop` | ODM orthophoto | calibrated pre-cut | Copernicus |
| `odm_dsm` | ODM orthophoto | calibrated pre-cut | ODM DSM |

The pre-cut calibration (`calibrate`) is re-run per configuration on photos 1–46 only.

## Reading the results

- `main_dsm` vs reference: effect of the fine terrain alone.
- `odm_cop` vs reference: effect of a closer, same-camera map (5 months instead of 8; same drone model).
- `odm_dsm`: best case (survey flight). Not representative of a mission without a survey flight.
- Same pass criteria; any improvement is reported with the per-seed spread, not only the pooled median.
