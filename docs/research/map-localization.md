# Localisation caméra → carte — rapport final

## Résumé en 15 lignes
1. **[MESURÉ + SIMULÉ]** Le banc principal couvre 18 sites, 20 conditions et 209 416 lignes de résultats (`r_map_benchmark_all/run.json`).
2. Sur les six sites NLSC 2015→2023, ZNCC brut est à ≤10 m pour 147/240 positifs alignés; quad≥3 en garde 50 et UNION 59 (`r_map_benchmark_all/matches.csv.gz`).
3. Sur tous les sites/conditions, ZNCC quad≥3 donne 1 016 fixes corrects/7 850 (1 017 acceptés) et 1 faux groupe/2 395 (borne 95 %: 0,198 %; `integrity_summary.csv`).
4. ZNCC lacet/échelle + quad≥3 donne 1 133 fixes ≤10 m/7 850 (1 135 acceptés), 0 faux groupe/2 395 (borne 0,125 %; même fichier).
5. UNION donne 1 878 fixes ≤10 m/7 850 (1 885 acceptés), 0/2 395 faux groupes (borne 0,125 %), mais la confirmation a un faux groupe rivière (`r_map_benchmark_confirm/`).
6. **[MESURÉ]** Sur ALTO réel, ZNCC lacet/échelle + quad≥3 accepte 44/300, médiane 10,44 m, maximum 17,70 m, 0 négatif accepté (`r_alto_matchers/frames.csv`).
7. XFeat affine accepte 0/300 images ALTO; XFeat homography accepte aussi un négatif (`r_alto_matchers/summary.csv`).
8. Le haze synthétique garde 129/395 fixes UNION corrects; le flou 9 px en garde 99/395, le flou 21 px 31/395 (`r_map_benchmark_all/integrity_summary.csv`).
9. UNION accepte 110/395 à lacet 30°, 126/395 à 90°; l’échelle ×1,25 tombe à 40/389 (`integrity_summary.csv`).
10. **[SIMULÉ]** La boucle finale couvre 19 sites, 4 seeds et 7 variantes; temps réel 46 min 25,6 s (`r_closed_loop_final/run.json`).
11. À 10 s de période et 60 s de blackout, union_driftgate réduit l’erreur médiane agrégée de 104,8 m (DR seul) à 17,6 m; 868 fixes acceptés, 0 faux (`summary.csv`).
12. Sans gate, accept_all accepte 2 557 fixes dont 1 043 à >25 m (`summary.csv`).
13. Avec 240 s de blackout, union_adaptive termine à 18,9 m d’erreur médiane finale, contre 220,7 m pour DR seul (`summary.csv`).
14. En recherche sans prior, le grand mosaïque aligné donne 40,6 % top‑1 ≤10 m et 23,8 % après quad≥3; le mosaïque sans site-source accepte 0/143 négatifs (`r_lost_mode/summary.csv`).
15. **Recommandation:** ZNCC + quad≥3 + gate d’innovation avec covariance de dérive; n’utiliser UNION qu’après validation rivière/eau sur images réelles indépendantes.

## Périmètre et étiquettes

- **[MESURÉ + SIMULÉ]** Les images carte sont de vraies orthophotos; les requêtes caméra des bancs `r_map_benchmark*`, `r_closed_loop*` et `r_lost_mode` sont rendues par le générateur pinhole. Les taux de ces bancs ne sont donc pas des taux de vol réel. Protocoles et géométrie: `data/processed/r_map_benchmark_all/run.json`, `r_closed_loop_final/run.json`, `r_lost_mode/run.json`.
- **[MESURÉ]** ALTO contient de vraies images et métadonnées de vol; les offsets du prior et les négatifs sont simulés. Le matcher ne reçoit que l’image et la fenêtre carte (`r_alto_matchers/run.json`).
- **[MESURÉ]** Les latences de `r_matchers` sont des mesures CPU sur 10 paires, pas une mesure d’énergie ou de calcul embarqué (`r_matchers/timing.csv`).
- Les fichiers de sortie donnent les chiffres détaillés; les citations ci-dessous indiquent le fichier dont provient chaque résultat.

## Méthodes comparées

| Méthode | Règle | Lecture des résultats |
|---|---|---|
| ZNCC local | Corrélation normalisée en translation sur une fenêtre locale, prior synthétique ±60 m. | Très économique; sensible au décalage temporel, à l’échelle et aux apparences répétitives (`r_map_benchmark_all/matches.csv.gz`). |
| ZNCC quad≥3 | Quatre sous-gabarits disjoints de 56 px; au moins trois centres doivent être à ≤4 px du centre ZNCC complet. | Contrôle non appris, bonne précision mais couverture limitée (`experiments/r_map_benchmark.py`, `r_map_benchmark_all/integrity_summary.csv`). |
| ZNCC lacet/échelle | Hypothèses de lacet −20°…+20° par pas de 10° et échelles 0,9/1/1,1; même contrôle quad. | Aide pour petites erreurs d’attitude/altitude, mais pas pour un changement d’échelle de 20–25 % (`experiments/r_map_benchmark.py`, `r_map_benchmark_all/integrity_summary.csv`). |
| XFeat affine / homographie | Correspondances XFeat, RANSAC, seuils géométriques d’échelle, dispersion et inliers. | Affine échoue sur ALTO; homographie a accepté un négatif sur ALTO (`r_alto_matchers/summary.csv`, `frames.csv`). |
| Accord ZNCC × XFeat | Deux familles doivent localiser à ≤10 m. | L’accord `zncc`/`xfeat_affine` est conservateur; un accord entre deux variantes ZNCC n’est pas une indépendance de méthode (`r_map_benchmark_all/integrity_summary.csv`, `false_accepts_by_land_cover.csv`). |
| UNION | Accepte un quad≥3 d’une variante ZNCC ou un accord inter-familles euclidien ≤10 m; l’enveloppe de tous les candidats admis reste ≤10 m par axe. | Meilleure couverture dans le run principal, mais faux positif rivière dans le run de confirmation (`experiments/r_integrity.py`, `r_map_benchmark_confirm/false_accepts_by_land_cover.csv`). |

## Intégrité et faux positifs

Définition utilisée par `experiments/r_integrity.py`: faux accept = négatif accepté **ou** positif dont l’erreur dépasse 25 m. Les conditions répétées d’une même fenêtre/scène sont regroupées par `site|pair|centre|kind`; borne supérieure unilatérale Clopper–Pearson à 95 %, sous l’hypothèse d’indépendance de ces groupes.

| Règle | Corrects ≤10 m / positifs | Faux groupes / groupes | Borne supérieure 95 % |
|---|---:|---:|---:|
| `zncc[quad>=3]` | 1 016/7 850 = 12,94 % (1 017 acceptés) | 1/2 395 | 0,198 % |
| `zncc_yaw_scale[quad>=3]` | 1 133/7 850 = 14,43 % (1 135 acceptés) | 0/2 395 | 0,125 % |
| `agree(zncc,xfeat_affine)` | 661/7 850 = 8,42 % (665 acceptés) | 0/2 395 | 0,125 % |
| `UNION` | 1 878/7 850 = 23,92 % (1 885 acceptés) | 0/2 395 | 0,125 % |

Source des quatre lignes: `data/processed/r_map_benchmark_all/integrity_summary.csv` (18 sites, 20 conditions). Le quad ZNCC a un positif à >25 m; UNION a zéro négatif accepté dans ce run.

**Run de confirmation distinct, mais de taille plus réduite:** `r_map_benchmark_confirm/integrity_summary.csv` agrège 1 957 positifs, 10 551 négatifs et 2 103 groupes sur six conditions. UNION y donne 397 fixes corrects/1 957 (20,29 %), mais 3 faux positifs (négatifs acceptés) dans **un** groupe de rivière: borne 0,225 %. La règle `agree(zncc,xfeat_affine)` y donne 165/1 957 (8,43 %) et 0/2 103 faux, borne 0,142 %. Les trois lignes rivière sont visibles dans `r_map_benchmark_confirm/false_accepts_by_land_cover.csv`. Ne pas présenter le zéro du run principal comme une garantie générale.

### Six sites NLSC

Cartes 2015 contre requêtes rendues depuis 2023; condition alignée, 40 centres par site. ZNCC brut est ≤10 m sur 147/240 positifs; quad≥3 n’en retient que 50/240 (20,83 %), UNION 59/240 (24,58 %). Les deux règles gated ont 0 faux groupe sur 1 679 groupes NLSC, borne 0,178 % (`r_map_benchmark_all/matches.csv.gz`, calcul d’intégrité sur ces lignes).

| Site NLSC | ZNCC brut ≤10 m |
|---|---:|
| Changhua rice | 17/40 |
| Guanyin coast | 15/40 |
| Kaohsiung port | 23/40 |
| Nantou hills | 35/40 |
| Taipei urban | 39/40 |
| Zhuoshui river | 18/40 |

Ces écarts montrent qu’un seuil ou taux unique n’est pas représentatif des six couvertures (`r_map_benchmark_all/matches.csv.gz`).

## ALTO: images réelles

`r_alto_matchers/run.json` documente 300 images positives échantillonnées sur le vol ALTO Val, des fenêtres négatives simulées à ≥600 m, un prior simulé ±60 m par axe et une carte à 0,6 m/px. Le matcher ne voit pas la vérité.

- `zncc_yaw_scale`, accepté seulement si quad≥3: **44/300** positifs acceptés; parmi eux 22/300 sont à ≤10 m, médiane acceptée 10,44 m, maximum 17,70 m; aucun >25 m et **0/300** négatifs acceptés (`r_alto_matchers/frames.csv`, `summary.csv`).
- `xfeat_affine`: **0/300** positifs acceptés. `xfeat_homography`: 2 positifs acceptés, mais aussi **1** négatif (`summary.csv`, `frames.csv`).
- ZNCC simple accepte 25/300 positifs après quad≥3; le résultat yaw/scale augmente la couverture à 44/300 (`summary.csv`).

La borne binomiale naïve à 95 % pour 0/300 serait 0,994 %; elle n’est pas une borne de vol indépendante, car les images viennent d’un même parcours corrélé.

## Robustesse aux effets caméra

`UNION`, 18 sites et conditions générées; fractions = fixes acceptés à ≤10 m / positifs. Les données par condition et faux groupes sont dans `r_map_benchmark_all/integrity_summary.csv`; toutes les lignes UNION du run principal ont 0 faux groupe.

| Condition | UNION correct ≤10 m |
|---|---:|
| Aligné | 133/395 = 33,7 % |
| Haze (0,55) | 129/395 = 32,4 % |
| Flou de mouvement 9 px / 21 px | 99/395 = 25,1 % / 31/395 = 7,8 % |
| Ombre locale | 28/395 = 7,1 % |
| Lacet 30° / 90° / 180° | 110/395 = 27,8 % / 126/395 = 31,6 % / 124/395 = 31,4 % |
| Tilt 10° / rectifié; tilt 20° / rectifié | 95/391 = 24,3 % / 107/391 = 27,1 %; 59/375 = 15,5 % / 110/381 = 28,6 % |
| Échelle ×1,25 | 40/389 = 10,3 % |

`UNION` comprend `zncc_heading` (recherche de cap par pas de 15°); la recherche lacet/échelle limitée à ±20° seule ne récupère plus de quad correct au-delà de 10°. La rectification du tilt est **SIMULÉE**, avec bruit d’attitude de 1°; ce n’est pas une mesure IMU réelle (`r_map_benchmark_dates/run.json`, `r_map_benchmark_all/integrity_summary.csv`). Le score s’effondre avec l’ombre, le flou fort et les erreurs d’échelle hors plage.

## Coût des matchers sur CPU

Mesures à un thread, 10 paires; latences et erreurs de localisation (`r_matchers/timing.csv`, `r_matchers/loc_error.csv`).

| Matcher | Latence médiane / p95 | Erreur médiane / p90 |
|---|---:|---:|
| XFeat MNN | 44 / 58 ms | 4,46 / 127,42 m |
| XFeat + LighterGlue | 149 / 179 ms | 3,84 / 32,95 m |
| TinyRoMa | 66 / 75 ms | 3,28 / 4,94 m |
| ALIKED + LightGlue | 558 / 593 ms | 3,17 / 3,86 m |
| DISK + LightGlue | 696 / 943 ms | 3,49 / 3,89 m |
| SIFT + LightGlue | 2 152 / 2 432 ms | 3,83 / 10,43 m |
| RoMa outdoor | 12 416 / 13 350 ms | 3,52 / 3,90 m |

Échantillon trop petit pour choisir un matcher seul. RoMa outdoor ne convient pas à une correction toutes les 10 s sur ce CPU; XFeat MNN a un p90 d’erreur élevé malgré sa latence faible. Le dossier `r_matchers_smoke/timing.json` n’a **aucune paire**: `n_pairs=0`, `IndexError`; ne pas citer ses latences.

## Boucle fermée et fuite de vérité

**[SIMULÉ]** Le test final couvre 19 sites, 4 seeds par site/configuration, 12 conditions de balayage et 7 variantes; 912 tâches, 76 trajectoires par ligne du résumé. Commande: `time .venv/bin/python experiments/r_closed_loop.py --output data/processed/r_closed_loop_final`. Temps mur mesuré: **46 min 25,6 s**; `run.json` indique 2 785,1 s (`r_closed_loop_final/run.json`).

Vérification de fuite: `Estimator` reçoit les incréments d’odométrie, l’état GNSS initial/avant coupure, les images, la carte et le décalage de navigation issu de l’attitude simulée; ses méthodes n’ont pas d’argument `truth` (`experiments/r_closed_loop.py`). Dans `run`, GNSS n’est injecté que si `t < GNSS_CUT` (20 s); après la coupure, la vérité sert au générateur de trajectoire/images et au scorer, pas à l’estimateur (`r_closed_loop_final/run.json`, `experiments/r_closed_loop.py`).

Mesures à 10 s entre corrections, blackout de 60 s commençant 60 s après la coupure GNSS. Les champs `median_err`, `p95_err` et `final_err` sont les médianes des métriques par trajectoire, pas des percentiles regroupés.

| Variante | Erreur médiane / médiane du P95 / finale | Fixes acceptés / tentatives | >25 m |
|---|---:|---:|---:|
| DR seul | 104,8 / 208,4 / 220,7 m | 0/2 584 | 0 |
| UNION fixe | 101,0 / 203,5 / 218,4 m | 442/2 584 | 0 |
| UNION + drift gate | 17,6 / 62,6 / 18,4 m | 868/2 584 | 0 |
| UNION adaptive | 23,4 / 80,4 / 17,8 m | 842/2 584 | 1 |
| Accept all, sans gate | 10,2 / 171,5 / 6,1 m | 2 557/2 584 | **1 043** |

Source: `r_closed_loop_final/summary.csv`; faux fix = accepté à >25 m (`WRONG=25` dans le script). La médiane très basse d’accept_all cache donc une forte proportion d’erreurs dangereuses.

| Blackout, période 10 s | UNION + drift gate: erreur médiane / finale | Acceptés / tentatives |
|---|---:|---:|
| 0 s | 15,3 / 18,6 m | 1 035/3 040 |
| 60 s | 17,6 / 18,4 m | 868/2 584 |
| 120 s | 21,8 / 18,6 m | 715/2 128 |
| 240 s | 61,5 / 22,6 m | 355/1 216 |

À 240 s, `union_adaptive` donne 58,8 m d’erreur médiane et 18,9 m finale (393/1 216 fixes); la fenêtre fixe perd davantage de corrections. Des faux fixes isolés existent ailleurs dans le balayage: union_driftgate a un >25 m sans blackout et sous `motion9`/`scale1.25` (`summary.csv`).

## Mode perdu, sans prior

**[MESURÉ + SIMULÉ]** 18 sites, mosaïque non géoréférencée de 23,31 km² (11 516×10 850 px), 288 tâches, conditions alignée et lacet 10°. Recherche: pyramide ×8, 40 pics grossiers, raffinement pleine résolution ±64 px (`r_lost_mode/run.json`). Temps mur mesuré **1 min 14,4 s**; `run.json` indique 71,5 s de balayage.

| Région de recherche | Positifs valides | Top‑1 ≤10 m, aligné | Quad≥3 correct, aligné | Vérifié XFeat correct, aligné |
|---|---:|---:|---:|---:|
| Fenêtre 384×384 px | 143 | 59,4 % | 26,6 % | 17,5 % |
| Fenêtre 768×768 px | 90 | 55,6 % | 22,2 % | 12,2 % |
| Fenêtre 1536×1536 px | 84 | 48,8 % | 23,8 % | 13,1 % |
| Site complet | 143 | 49,7 % | 26,6 % | 18,2 % |
| Mosaïque 23,31 km² | 143 | 40,6 % | 23,8 % | 17,5 % |

Dans `mosaic_without_site`, les images-source sont absentes; **0/143** négatifs sont acceptés par quad≥3 ou XFeat vérifié, pour chacune des deux conditions (`r_lost_mode/summary.csv`, `lost_mode.csv`). La médiane ZNCC sur mosaïque est 96,8 ms et la vérification XFeat 282,0 ms. Échantillon modeste; pas de borne statistique fiable. Les fenêtres 384/768/1536 sont tirées de façon à contenir la vérité, donc seule la mosaïque teste le vrai global sans prior.

## Inventaire des sorties `r_*`

Les compteurs des sorties et tous les fichiers cités sont sous `data/processed/`. Les anciens `run.json` n’enregistrent pas toujours l’argv complet; les commandes de reproduction épinglées plus bas reprennent les sites, seeds, méthodes et conditions conservés dans les sorties.

| Dossier | Commande de reproduction | Résultat tête et fichier |
|---|---|---|
| `r_map_benchmark/` | `C1` | 7 sites, 163 152 lignes, 1 188,0 s; quad: 584/5 297 fixes corrects, 585 acceptés, 1/1 855 faux groupe. `integrity_summary.csv`, `run.json`. |
| `r_map_benchmark_all/` | `C2` + intégrité | 18 sites, 209 416 lignes, 1 727,2 s; banc principal 20 conditions. `run.json`, `integrity_summary.csv`, `matches.csv.gz`. |
| `r_map_benchmark_confirm/` | `C3` + intégrité | 18 sites, seed 777, 75 048 lignes, 1 779,2 s; UNION a accepté 3 négatifs comme fixes, sur 1 groupe rivière. `run.json`, `integrity_summary.csv`, `false_accepts_by_land_cover.csv`. |
| `r_map_benchmark_dates/` | `C4` | 6 dates, 5 paires depuis 2018, 6 632 lignes, 63,5 s; 2018→2019‑12: ZNCC ≤10 m sur 12/28, erreur médiane 66,82 m. `matches.csv.gz`. |
| `r_map_benchmark_wufeng/` | `C5` + intégrité | 1 site, 15 352 lignes, 118,2 s; quad: 195/499 fixes corrects, 0/175 faux groupes (borne 1,697 %). `integrity_summary.csv`, `run.json`. |
| `r_map_benchmark_smoke/` | `C6` | 24 lignes, 3,8 s; smoke seulement, pas de conclusion statistique. `run.json`, `matches.csv.gz`. |
| `r_matchers/` | `C7` | 10 paires; coûts et erreurs détaillés ci-dessus. `timing.csv`, `loc_error.csv`, `benchmark_run.log`. |
| `r_matchers_smoke/` | argv ancien non conservé | 0 paire; tous les appels échouent par `IndexError`. `timing.json`. |
| `r_alto_matchers/` | `C8` | 300 images réelles; ALTO détaillé ci-dessus. `summary.csv`, `frames.csv`, `run.json`. |
| `r_closed_loop/` | `C9` | Ancien balayage 7 sites × 4 seeds; 1 697,6 s. Supplanté par le run complet ci-dessous. `summary.csv`, `run.json`. |
| `r_closed_loop_smoke/` | `C10` | 1 site, 3 seeds, UNION et UNION adaptive; à 240 s de blackout, erreurs médianes 57,4 m et 21,2 m. `summary.csv`, `run.json`. |
| `r_closed_loop_final/` | `C11` | Exécution complète et mesure de runtime; résultats finaux ci-dessus. `summary.csv`, `runs.csv`, `run.json`. |
| `r_lost_mode/` | `C12` | Recherche globale sans prior; résultats finaux ci-dessus. `summary.csv`, `lost_mode.csv`, `run.json`. |

## Commandes reproductibles

```bash
# C1 — sept sites NLSC + Wufeng
.venv/bin/python experiments/r_map_benchmark.py \
  --sites nlsc_changhua_rice,nlsc_guanyin_coast,nlsc_kaohsiung_port,nlsc_nantou_hills,nlsc_taipei_urban,nlsc_zhuoshui_river,oam_wufeng \
  --seed 20261003 --workers 8 --max-centres 40 --neg-same 6 --neg-other 2 \
  --output data/processed/r_map_benchmark
.venv/bin/python experiments/r_integrity.py data/processed/r_map_benchmark

# C2 — 18 sites, méthodes/conditions par défaut
.venv/bin/python experiments/r_map_benchmark.py \
  --sites nlsc_changhua_rice,nlsc_guanyin_coast,nlsc_kaohsiung_port,nlsc_nantou_hills,nlsc_taipei_urban,nlsc_zhuoshui_river,oam_113498,oam_20a26d,oam_20a2ac,oam_221052,oam_4c92fa,oam_4c9306,oam_4c932a,oam_4c932e,oam_61702e,oam_c7cc77,oam_e6abbd,oam_wufeng \
  --seed 20261003 --workers 8 --max-centres 40 --neg-same 6 --neg-other 2 \
  --output data/processed/r_map_benchmark_all
.venv/bin/python experiments/r_integrity.py data/processed/r_map_benchmark_all

# C3 — répétition seed 777 avec matchers additionnels et six conditions
.venv/bin/python experiments/r_map_benchmark.py \
  --sites nlsc_changhua_rice,nlsc_guanyin_coast,nlsc_kaohsiung_port,nlsc_nantou_hills,nlsc_taipei_urban,nlsc_zhuoshui_river,oam_113498,oam_20a26d,oam_20a2ac,oam_221052,oam_4c92fa,oam_4c9306,oam_4c932a,oam_4c932e,oam_61702e,oam_c7cc77,oam_e6abbd,oam_wufeng \
  --seed 777 --workers 8 --max-centres 40 --neg-same 6 --neg-other 2 \
  --methods zncc,zncc_yaw_scale,xfeat_affine,lib:tiny_roma,lib:aliked_lightglue,lib:xfeat_lighterglue \
  --conditions aligned,legacy_degraded,motion9,scale1.25,tilt10_rect,yaw10 \
  --output data/processed/r_map_benchmark_confirm
.venv/bin/python experiments/r_integrity.py data/processed/r_map_benchmark_confirm

# C4 — paires temporelles OAM; C5 — Wufeng seul; C6 — smoke de deux unités
.venv/bin/python experiments/r_map_benchmark.py --sites oam_e9d0dc --pairs-mode oldest --seed 20261003 --workers 4 --max-centres 40 --neg-same 3 --methods zncc,zncc_yaw_scale,xfeat_affine,lib:tiny_roma --conditions aligned,legacy_degraded,motion9 --output data/processed/r_map_benchmark_dates
.venv/bin/python experiments/r_map_benchmark.py --sites oam_wufeng --seed 20261003 --workers 8 --max-centres 40 --neg-same 6 --output data/processed/r_map_benchmark_wufeng
.venv/bin/python experiments/r_integrity.py data/processed/r_map_benchmark_wufeng
.venv/bin/python experiments/r_map_benchmark.py --sites oam_wufeng --limit-units 2 --workers 2 --conditions aligned --methods zncc,lib:tiny_roma,lib:aliked_lightglue,lib:xfeat_lighterglue --output data/processed/r_map_benchmark_smoke

# C7–C8 — coût matcher et images ALTO
.venv/bin/python experiments/r_matchers.py --threads 1 10 --loc-error
.venv/bin/python experiments/r_alto_matchers.py

# C9–C12 — boucles et mode perdu
.venv/bin/python experiments/r_closed_loop.py --sweep default --seeds 4 --workers 5 --sites oam_wufeng,nlsc_changhua_rice,nlsc_guanyin_coast,nlsc_kaohsiung_port,nlsc_nantou_hills,nlsc_taipei_urban,nlsc_zhuoshui_river --output data/processed/r_closed_loop
.venv/bin/python experiments/r_closed_loop.py --sweep default --seeds 3 --workers 4 --sites oam_wufeng --variants union,union_adaptive --output data/processed/r_closed_loop_smoke
time .venv/bin/python experiments/r_closed_loop.py --output data/processed/r_closed_loop_final
time .venv/bin/python experiments/r_lost_mode.py --workers 8
```

## Échecs, recommandation et limites

### Ce qui a échoué

- **UNION n’est pas encore un seuil de sûreté:** zéro faux sur le run principal, puis un groupe de faux positifs sur images négatives en rivière dans la confirmation (`r_map_benchmark_confirm/false_accepts_by_land_cover.csv`).
- L’accord de deux variantes ZNCC est corrélé et produit des faux accept; les scores seuls ne suffisent pas. UNION_v1, qui ajoute la géométrie homographique, ajoute aussi un faux groupe urbain (`r_map_benchmark_all/false_accepts_by_land_cover.csv`).
- XFeat affine seul n’accepte aucune image ALTO; l’homographie prend un négatif. Accept_all donne 1 043 fixes à >25 m au balayage nominal fermé.
- Les grands écarts d’échelle et le flou/les ombres réduisent fortement la couverture gated; le quad devient presque muet à lacet 10° si l’hypothèse de cap n’est pas explorée.
- Le balayage perdu pleine résolution dépassait 3 600 s sans produire de fichier. Le résultat final emploie une recherche grossière puis raffinée, documentée dans `r_lost_mode/run.json`; elle a terminé en 74,4 s. Le matcher smoke n’avait aucune paire.

### Recommandation rangée pour le module carte

1. **Prototype principal:** ZNCC local avec quatre sous-gabarits et acceptation quad≥3; gate d’innovation χ² à 99 %, covariance augmentée par la dérive depuis le dernier fix et fenêtre de recherche adaptée à 3σ (384–1 024 px). Rejeter le fix plutôt que forcer une correction.
2. **Hypothèses caméra:** tester une banque lacet/échelle bornée et la rectification tilt uniquement avec attitude/altitude effectivement disponibles et calibrées; les hypothèses testées ne couvrent pas ×0,8 ou ×1,25.
3. **XFeat:** le garder comme vérification facultative sur des domaines où il est validé; ne pas le rendre obligatoire, puisque l’affine fait 0/300 sur ALTO. Revalider UNION sur la rivière avant de l’utiliser comme porte principale.
4. **Ne pas embarquer comme primaire** RoMa outdoor (12,4 s/pair mesuré ici), XFeat homography sans validation supplémentaire, accord entre variantes ZNCC seulement, ou acceptation du top‑1 sans gate.

### Limites et prochaines étapes

- Les cartes OAM sont sous CC‑BY 4.0 avec attribution; certains éléments sont des vols UAV de petite emprise, pas de grandes orthophotos. Les étiquettes de couverture sont heuristiques (`data/raw/aerial_pairs/manifest_oam.json`). Vérifier les droits de cache et redistribution hors ligne des tuiles NLSC avant déploiement.
- Les requêtes des bancs cartographiques et du mode perdu sont rendues depuis des cartes; elles ne reproduisent ni caméra, ni vibrations, ni occultation, ni variation de terrain complète. ALTO est un seul domaine/vol et les images successives sont corrélées.
- Les bornes Clopper–Pearson supposent des groupes indépendants; les fenêtres voisines peuvent rester corrélées. Le faux groupe rivière de confirmation prouve que 0 faux observé n’est pas une garantie opérationnelle.
- Le mode perdu utilise un mosaïque de sites disjoints et des fenêtres positives contenant la vérité par construction; ce n’est pas une carte géoréférencée continue de Taïwan.
- Étapes suivantes: valider sur des prises caméra réelles à dates distinctes; constituer des négatifs rivière/eau/texture répétitive; calibrer sans réutiliser la scène test; tester la chaîne complète sur le calculateur embarqué et faire confirmer les licences de tuiles hors ligne.
