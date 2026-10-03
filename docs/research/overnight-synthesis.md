# Synthèse de la nuit du 2 au 3 octobre 2026

Branche locale `research/offline-nav-evidence` (depuis `origin/main` 1286d5c). Rien n’est commité ni poussé. Les rapports détaillés sont dans ce dossier :

- `map-localization.md` : piste A, recalage caméra → carte ;
- `sensor-fusion.md` : piste B, hauteur, vitesse et cap ;
- `datasets-replay-sim.md` et `data-manifest.md` : piste C, données, rejeu et simulateur ;
- `../../questions.md` : questions pour l’équipe.

Étiquettes : **MESURÉ** (lancé ici sur données réelles), **SIMULÉ** (générateur déclaré), **PUBLIÉ** (source citée), **INFÉRENCE**.

## 1. En cinq points

1. **Les 26–31 m d’ALTO Val ne tiennent pas sur un vol jamais vu.** Sur ALTO Round 2 Train (37,4 km réels, protocole pré-enregistré, paramètres gelés), la chaîne de l’équipe donne une médiane par section de 94 m avec un recalage tous les 300 m (20–339 m selon la section), contre 31 m sur Val. Elle ne reproduit Val que dans 3 sections sur 8. Les chiffres de Val sont à présenter comme **réglés sur le jeu de test**. [MESURÉ, `datasets-replay-sim.md`]
2. **La cause principale est l’échelle de l’image (zoom) calibrée sur trois recalages**, pas l’appariement lui-même. Avec un meilleur zoom, la part de recalages justes passe de 21–43 % à 86–100 % dans trois sections. Le score de corrélation est maximal au bord de la grille de zoom quand il se trompe. [MESURÉ, diagnostic qui utilise la vérité pour l’évaluation seulement]
3. **Un contrôle d’intégrité sans seuil appris marche mieux qu’un seuil de score.** « Quad ≥ 3 » : quatre sous-gabarits disjoints doivent retomber au même endroit. Sur 300 vraies images ALTO, ZNCC avec recherche de lacet et d’échelle + quad ≥ 3 accepte 44 recalages, médiane 10,4 m, pire 17,7 m, 0 faux, 0 négatif accepté sur 300. Les seuils de score, eux, ne se transfèrent pas d’une moitié de carte à l’autre. [MESURÉ, piste A]
4. **Le modèle appris XFeat ne sert pas sur les vraies images aériennes testées** : 0/300 sur ALTO, 1/60 sur un site OrthoLoC jamais vu. Il marche sur des recadrages d’orthophotos (Wufeng) mais s’effondre avec le flou de bouger et un lacet de 30° ou plus. [MESURÉ]
5. **Capteurs en plus** : un vrai baromètre tient la hauteur relative à 0,6 m (médiane) après 60 s et 1,8 m après 5 min. Le capteur solaire réduit l’erreur en travers mais seulement de 6 % l’erreur finale sur ALTO. La stéréo du commerce ne sert que sous environ 30–90 m de hauteur sol. Le plus gros levier mesuré reste la calibration de la vitesse caméra (74 m en fin de section si on la connaissait sur tout le trajet, contre 608 m). [MESURÉ et SIMULÉ, piste B]

## 2. Proposition pour l’équipe

Classée de la plus sûre à la plus spéculative.

| Rang | Élément | Pourquoi | Preuve |
|---|---|---|---|
| 1 | **Recalage ZNCC avec recherche de lacet et d’échelle, image redressée avec l’attitude de l’IMU** | Seule méthode qui marche sur vraies images (ALTO) ; sans redressement, 0 succès sur OrthoLoC à 21° de visée oblique | MESURÉ (A, V3) |
| 2 | **Intégrité par cohérence interne (quad ≥ 3), sans seuil de score ; accord entre méthodes dissemblables seulement en complément** | ZNCC + lacet/échelle + quad ≥ 3 : 0 faux sur 2 395 groupes négatifs à 18 sites (borne haute 95 % : 0,125 %). Les seuils de score acceptent des négatifs hors zone de réglage. Pas un zéro universel : la règle UNION a laissé passer un négatif de rivière, et 3 négatifs sur 3 540 paires sur le site OrthoLoC | MESURÉ sur orthophotos réelles, caméra simulée (A) ; vraies images (V3) |
| 3 | **Échelle du recalage tirée de la hauteur sol (baro − modèle d’élévation), avec une grille de zoom large, toujours combinée au contrôle quad ≥ 3** | Corrige la cause n° 1 de l’échec hors réglage. Sur Round 2 (exploratoire, § 6) : seule, elle double les faux recalages ; avec quad ≥ 3, médiane des sections 70 m au lieu de 128 m et aucun faux | MESURÉ (C, V1) ; SIMULÉ pour le baro |
| 4 | **Baromètre pour la hauteur, gyroscope pour le cap entre recalages** | Baro réel : 0,30 m de bruit, 0,112 m/√s de marche aléatoire ; l’IMU n’améliore pas la dérive de hauteur | MESURÉ (Zurich, INSANE, journaux PX4 RTK) |
| 5 | **Budget de dérive déclaré quand il n’y a plus de recalage** (mer, forêt, nuit) | Sans recalage : 219 m de médiane par section de 4,6 km sur Round 2 ; aucun indice testé cette nuit ne le remplace | MESURÉ |
| 6 | Capteur solaire à fente (option) | Erreur en travers 196 → 31 m sur ALTO, mais −6 % sur l’erreur finale ; demande soleil sous 70°, ciel clair (40 % du jour à Taichung en octobre) et attitude à 0,5° | SIMULÉ sur trajet réel (B) |
| 7 | Stéréo (option, basse altitude seulement) | Base de 0,30 m : 1,3 px de disparité à 60 m | SIMULÉ (C, B) |

## 3. Ce qui a été essayé et abandonné

Chaque essai avait un critère d’arrêt écrit avant le lancement.

| Idée | Résultat | Fichier |
|---|---|---|
| Boussole d’ombres sur l’image | Dispersion 23° (gradient) et 51° (silhouettes), critère 10° : abandon | `experiments/u1_shadow_compass.py` |
| Échelle baro − MNE sans recalage | Erreur finale 639 m contre 608 m ; médiane 372 contre 472 m ; même une hauteur parfaite laisse 426 m | `u2_baro_dem_scale.py` |
| Recalage sur routes OpenStreetMap (chamfer) | Après correction d’un bogue, la vraie position n’est même pas un minimum local : abandon | `u5_osm_fix.py` |
| Recalage sur le trait de côte (Sentinel-2) | Aucun site validé ; port de Taichung prometteur (4 m) mais 0/3 accepté ; vasières de Changhua faussées par la marée (marnage 5,4 m) | `u3_coastline.py` |
| Vitesse par traînée + vent sur journaux PX4 | Pire que garder la dernière vitesse GNSS en moyenne sur 3 vols ; identification mal posée | `u6_px4_drag.py` |
| Anomalie magnétique (EMAG2v3) | Précision médiane 9,6 km au-dessus du détroit : abandon | `u7_magnav_kill.py` |
| Calibration en ligne de l’échelle et du cap par les recalages | Échec pré-enregistré : 162 m à 1 000 m d’espacement contre 56 m pour l’équipe | `s_alto_online_calib.py` |
| Modèles appris (XFeat) sur vraies images | 0/300 ALTO, 1/60 OrthoLoC | `r_alto_matchers.py`, `v3_ortholoc_heldout.py` |
| Altitude tirée de la vidéo à la place d’un baro | Sous-estime la dérive d’un facteur 2,7 à 3 : ne pas l’utiliser pour évaluer | `sensor-fusion.md` |

## 4. Où chaque indice peut marcher à Taïwan

Carte `data/processed/u8_taiwan_coverage/coverage_map.png`. MNE Copernicus GLO-30 et occupation du sol ESA WorldCover, réels. La règle « recalage probable » est une INFÉRENCE (sol texturé hors forêt et eau) ; l’erreur de hauteur de 4 m est une hypothèse.

| Zone | Recalage carte probable | Relief informatif (empreinte 420 m) | Aucun des deux |
|---|---|---|---|
| Île principale | 21 % | 72 % | 9 % |
| Bande côtière de 20 km | 33 % | 55 % | 14 % |
| Taipei | 20 % | 81 % | 4 % |
| Taichung | 45 % | 50 % | 9 % |
| Kaohsiung | 47 % | 17 % | 37 % |

La forêt couvre 76 % de l’île : le recalage sur orthophoto y est improbable, le recalage sur le relief y devient l’indice principal. Le recalage sur le relief n’a pas été testé en vol cette nuit.

## 5. Données sur disque (ignorées par git)

Détail et licences : `data-manifest.md`.

- ALTO Val (1,86 Go) et Round 2 Train (11,3 Go), récupérés par liens Dropbox fichier par fichier.
- Zurich Urban MAV : journaux complets, échantillon et fenêtre de 600 s d’images (caméra vers l’avant, inutile pour la carte).
- INSANE (caméra vers le bas, baro brut, RTK), 26 journaux PX4 dont 2 avec RTK, MUN-FRL (échantillon), OrthoLoC (site jamais vu, 60 images).
- Orthophotos Wufeng 2018/2020, échantillons NLSC et OpenAerialMap, Sentinel-2, OSM Taïwan, EMAG2v3, Copernicus, WorldCover.
- XFeat (code et poids, Apache-2.0). Image Docker du simulateur (4,8 Go).

## 6. Derniers essais de la nuit

### Piste A finalisée (`map-localization.md`)

- **Banc élargi** [MESURÉ sur orthophotos réelles, caméra SIMULÉE] : 18 sites, 20 conditions. Sur 7 850 positifs, ZNCC + lacet/échelle + quad ≥ 3 en accepte 1 133 justes, sans faux sur 2 395 groupes négatifs (borne haute 95 % : 0,125 %). La règle UNION en accepte 1 878. Mais une confirmation a trouvé un faux sur un négatif de rivière : **UNION n’est pas une garantie**, à revalider sur l’eau.
- **Robustesse** : brume, lacet jusqu’à 180° et inclinaison redressée de 20° restent exploitables. Un flou de bouger de 21 px et une erreur d’échelle de 25 % font chuter l’acceptation.
- **Boucle fermée** [SIMULÉ, 19 sites, 4 tirages] : avec 60 s sans recalage, la variante à porte de dérive accepte 868 recalages sur 2 584, sans faux. L’erreur médiane passe de 104,8 m (estime seule) à 17,6 m. Sans porte, 1 043 des 2 557 recalages acceptés sont faux de plus de 25 m. Avec 240 s sans recalage, la variante adaptative finit à 18,9 m contre 220,7 m pour l’estime seule.
- **Mode perdu, sans a priori** : sur une mosaïque de 23,3 km², 40,6 % des meilleurs candidats sont à 10 m près en visée alignée, et 0/143 négatif accepté après la porte.
- Ces chiffres viennent d’images d’orthophotos découpées. Sur vraies images, les seuls chiffres sont ALTO (44/300 acceptés) et OrthoLoC (§ 6, V3).

### Site jamais vu : OrthoLoC `test_outPlace/L08` (V3)

60 vraies images de drone à environ 100 m au-dessus du sol, visée inclinée de 21 à 23°, avec orthophoto, MNS et pose par image (licence CC BY-NC-SA 4.0). Aucun réglage sur ce site. Redressement avec une attitude perturbée de 1° par axe et une hauteur à ±3 % (SIMULÉ). A priori de position à ±40 m. `experiments/v3_ortholoc_heldout.py`.

| Méthode | Juste à 10 m (brut) | Acceptés | Acceptés faux | Négatifs acceptés / 3 540 |
|---|---|---|---|---|
| ZNCC | 26/60 | 3 | 0 | 1 |
| ZNCC + lacet/échelle | 25/60 | 5 | 0 | 3 |
| XFeat + RANSAC | 1/60 | 1 | 0 | 0 |
| Règle UNION | 5/60 | 5 | 0 | 3 |
| ZNCC sans redressement | 3/60 | 0 | 0 | 0 |

Lecture : sans redressement par l’attitude, rien ne marche ; avec, l’appariement trouve juste 4 fois sur 10, mais le contrôle d’intégrité n’en garde qu’une sur dix.

### Intégrité et zoom sur Round 2 (V1, exploratoire)

**Pas tenu à l’écart** : les 8 sections avaient déjà été vues par le diagnostic, et il ne reste aucun vol ALTO avec vérité jamais regardé. Baro SIMULÉ (altitude ALTO + résidus réels de Zurich), MNE Copernicus. `experiments/v1_round2_integrity.py`, 84 min.

| Recalage tous les | Variante | Médiane des médianes de section | Médiane des erreurs finales | Recalages utilisés / rejetés / faux > 50 m |
|---|---|---|---|---|
| 300 m | chaîne gelée (recherche dimensionnée + seuil 0,33) | 128 m | 833 m | 35 / 89 / 6 |
| 300 m | + quad ≥ 3 | 126 m | 839 m | 13 / 111 / 0 |
| 300 m | + zoom baro − MNE | 406 m | 975 m | 30 / 56 / 12 |
| 300 m | + quad ≥ 3 + zoom baro − MNE | **70 m** | **296 m** | 10 / 108 / 0 |
| 1 000 m | chaîne gelée | 160 m | 919 m | 13 / 21 / 2 |
| 1 000 m | + quad ≥ 3 + zoom baro − MNE | 118 m | 479 m | 3 / 28 / 0 |

Lecture : le contrôle quad ≥ 3 supprime les faux recalages ; le zoom tiré du baro n’aide qu’avec lui. Le prix : un recalage tenté sur dix seulement est accepté. Le prochain gain viendra d’un appariement qui accepte davantage sans faux, à valider sur un vol réellement neuf (MUN-FRL avec orthophoto, MARS-LVIG, ou un vol de l’équipe).

## 7. Limites

- Pas un seul vol réel avec caméra vers le bas, baro brut, IMU et orthophoto du même lieu. Chaque preuve couvre une partie de la chaîne.
- Aucun vol à Taïwan avec baro brut trouvé (recherche bornée, YouTube et requêtes en chinois comprises). Les vidéos DJI donnent une altitude fusionnée par le constructeur.
- Les essais sur orthophotos découpent des images d’en haut : pas de perspective ni de vraie caméra, sauf ALTO et OrthoLoC.
- Rien n’a tourné sur une carte embarquée ; les temps sont mesurés sur un cœur de Mac.

## 8. Pour reprendre

```bash
cd TaipeiDrift && uv sync --extra research
.venv/bin/python experiments/t_alto_heldout.py        # tenu à l'écart ALTO Round 2 (voir son --help)
.venv/bin/python experiments/r_alto_matchers.py       # appariements sur 300 vraies images ALTO
.venv/bin/python experiments/p_zurich_baro.py         # baromètre réel contre photogrammétrie
.venv/bin/python experiments/u8_taiwan_coverage.py    # carte de couverture de Taïwan
```

Le simulateur : `data/processed/sim_smoke/smoke_report.txt` (contrôle des capteurs réussi, facteur temps réel 0,99) et le correctif proposé `data/processed/t_sim_rec/sim_patch.diff`. Le correctif retire la fuite d’attitude vraie sur `/imu/data`, ajoute la coupure GNSS, l’enregistreur et une deuxième caméra.

## 9. Organisation de la nuit

- Opus a orchestré et réfléchi ; trois chefs de piste Opus ont délégué recherche, téléchargements et code à des agents GPT-6 Luna.
- Un agent Haiku a surveillé le quota Claude.
- Featherless DeepSeek a été essayé en premier : la limite de 32 768 jetons par requête de l’abonnement l’a rendu inutilisable pour des agents outillés. Il reste en secours.
- Configuration propre à ce dossier : `../../.omp/config.yml` et `../../.omp/agents/`, hors du dépôt git.
