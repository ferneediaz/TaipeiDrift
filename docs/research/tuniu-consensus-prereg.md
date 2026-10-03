# Pre-registration: XFeat + ZNCC consensus gate (Tuniu, Level 1)

Written 2026-10-03 ~17:10 Taipei, BEFORE any run of this rule on seeds 1–19 and before the XFeat seed
1–19 test results were read. Committed to git so the timestamp is verifiable.

## Where the idea comes from (honest disclosure)

Exploratory, post-hoc look at the existing seed-0 test rows of the step-1 run
(`data/processed/x_tuniu/stage3_rows_main_test.csv`, main map OAM 2019-12-12, default query config
`h=baro_dem,att=dji,gnd=dem_prior`). Rule "XFeat and ZNCC fixes agree within 4 m, no other gate":

| res | consensus: accepted / wrong > 10 m / negatives | XFeat gate alone | ZNCC + quad ≥ 3 alone |
|---|---|---|---|
| 0.25 m | 70/225 (31.1 %) / 0 / 0 of 675 | 79 (35.1 %) / 0 / 0 | 57 (25.3 %) / 0 / 0 |
| 0.5 m | 70/225 (31.1 %) / 0 / 0 of 675 | 98 (43.6 %) / **7** / 1 | 55 (24.4 %) / 0 / 0 |
| 1.0 m | 29/225 (12.9 %) / 0 / 0 of 675 | 67 (29.8 %) / **10** / 0 | 62 (27.6 %) / 0 / 0 |

Seed 0 informed the rule, so seed 0 is EXCLUDED from its evaluation.

## Rule (fixed now)

- Same rectified query, same reference window, same settings as `data/processed/x_tuniu/preregistration.md`.
- Run `xfeat` (vismatch, 2048 keypoints, USAC_MAGSAC + plausibility checks) and `zncc` (yaw {−4,−2,0,2,4}°,
  scale {0.94,1,1.06}) independently.
- ACCEPT iff both return a fix and the two camera-nadir positions are within **4.0 m** (the quad tolerance
  already used by ZNCC + quad ≥ 3; not fitted). Reported position = the ZNCC fix.
- No score, inlier or quad threshold. Nothing is fitted, so the pre-cut photos are not needed for gates.

## Evaluation

- Seeds 1–19, test photos 47–271 (225 per seed), main map, default query config, resolutions 0.25 m and
  0.5 m. Negatives exactly as in the step-1 protocol (675 per seed).
- Optional, if time allows: the same at 0.5 m with `gnd=dem_lifted`.
- Report per resolution: accepted % (mean, min, max over seeds), pooled accepted wrong > 10 m with the exact
  95 % upper bound, pooled negatives accepted with the exact 95 % upper bound, median / p90 error, seeds
  passing the step-1 Level-1 criteria (≥ 30 % accepted, 0 wrong > 10 m, ≤ 1 % negatives).
- Success of the RULE = pooled wrong rate ≤ that of ZNCC + quad ≥ 3 at 1 m `gnd=dem_lifted` (3 / 1,461) with
  a mean accepted % ≥ 30 %. Otherwise it is reported as not better.

## Not claimed

One site, one flight, simulated ±40 m prior, DJI GNSS-aided attitude. A model or gate choice needs
confirmation on another site with everything frozen.
