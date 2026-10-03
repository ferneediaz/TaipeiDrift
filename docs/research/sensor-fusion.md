# Piste B : hauteur, vitesse et cap (rapport de nuit)

Étiquettes : **MESURÉ** (données réelles, lancé ici), **SIMULÉ** (générateur déclaré), **PUBLIÉ** (source avec lien), **INFÉRENCE**.
Toutes les commandes se lancent depuis la racine du dépôt avec `.venv/bin/python`.

## Recommandation classée (à lire en premier)

Constat qui commande tout le reste (MESURÉ sur la caméra ALTO, oracles et capteurs simulés, section 7) : sur les 608 m d'erreur finale d'ALTO, une hauteur sol parfaite en retire 30 %, un cap parfait 6 %, les deux 39 %. **Il reste 367 m le long de la route même avec hauteur et cap parfaits** : les pas caméra sont 10 % trop courts parce que la calibration flux → sol est apprise sur les seuls 300 premiers mètres. Calibrée sur tout le trajet (diagnostic, vérité utilisée), l'erreur tombe à 74 m. La plus grosse source d'erreur est donc la calibration de la vitesse caméra, pas un capteur manquant.

1. **Baromètre + modèle d'élévation (DEM), avec recalages carte** : 0 $, 0 W. Sur ALTO avec un recalage tous les 300 m, il fait passer les recalages ratés (erreur > 60 m à l'arrivée) de 32 % à 25 % (oracle : 19 %). Sans recalage il n'apporte rien (639 m contre 608 m) : le DEM est lu à une position déjà fausse. En plaine (type Wufeng), rien non plus. MESURÉ (caméra) + SIMULÉ (baro calé sur de vrais baros, recalages).
2. **Gyroscope déjà à bord pour le cap** : 0 $, 0 W. En simulation, dérive transversale 161 → 74 m sur 4,6 km si son biais est estimé avant la coupure à 0,005 °/s près (hypothèse). Pas testable sur ALTO (pas d'IMU publique, base de temps inconnue). SIMULÉ.
3. **Capteur solaire** : ≈ 150 $ en matériel libre (Foresail), 0,01 W. Sur ALTO, un capteur à ≈ 1° retire 84 % de la dérive transversale (196 → 31 m), mais seulement 6 % de l'erreur finale tant que la vitesse caméra n'est pas corrigée. Il faut une centrale à 0,5° d'inclinaison près (à 2°, il fait pire que le gyro en simulation), un soleil sous ≈ 70° d'élévation (inutilisable de mai à août vers midi) et un ciel dégagé (≈ 40 % du jour à Taichung en octobre, 12 % à Taipei en hiver). MESURÉ (caméra) + SIMULÉ + PUBLIÉ (CWA).
4. **Stéréo du commerce** : 230-780 $, 2,5-5,5 W. Utile seulement sous 27-92 m au-dessus du sol (5 % d'erreur de hauteur). Une OAK-D LR fait pire que baro + DEM à 120 m. Seule une base large (0,3-1 m, sur mesure) couvre 100-300 m. SIMULÉ, specs PUBLIÉES.
5. **L'IMU n'améliore pas la hauteur à long terme** : avec une vraie IMU à 196 Hz, elle lisse le baro sans retard (bruit ÷ 1,6) mais ne change pas la dérive à 60 s. MESURÉ (INSANE).

Prochaine action (≈ 2 min) : lancer `python experiments/s_alto_heading.py` et lire la ligne `true_agl heading_oracle` : c'est le plancher d'erreur que seuls les recalages ou une meilleure vitesse caméra peuvent battre.

---

## 1. Revue de `experiments/n_sensor_fusion.py` (SIMULÉ)

Commande : `python experiments/n_sensor_fusion.py --quick` (3 s), puis `python experiments/n_sensor_fusion.py` (17 s, 20 vols par régime, 600 s après la coupure). Sorties : `data/processed/sensor_fusion/`. 22 invariants sur 22 passent.

Fuite de vérité : aucune trouvée. Les estimateurs lisent seulement baro, accéléromètre, disparité, GNSS avant la coupure, roulis/tangage AHRS bruités ; le test de causalité (perturber l'après-coupure ne change pas l'avant) passe. Le DEM n'existe que dans une ablation étiquetée, lu à une position horizontale simulée avec erreur.

Physique vérifiée : disparité = f·B·cos(inclinaison)/hauteur le long de l'axe ; jacobien du filtre correct ; rotations NED→corps et corps→horizontal correctes ; éphémérides NOAA ; espace nul (z+c, b−c, h+c) de baro+stéréo+IMU vérifié (rang 4 sur 5).

Bogues corrigés :

| Bogue | Effet avant correction | Correction |
|---|---|---|
| Initialisation : terrain non corrélé à l'altitude | dès la première mise à jour GNSS, la hauteur sol estimée devenait négative ; la stéréo n'était plus jamais acceptée (0 trame acceptée) ; erreurs de 50 à 240 m partout | covariance initiale h = z − hauteur stéréo (cov(z,h) = var z) |
| Bruit de marche du terrain trop faible (pente 0,05) | sur relief, la porte de validation rejetait toutes les trames : hauteur sol fausse de 26 m (RMSE) contre 4,4 m pour la stéréo seule | pente 0,2 (≈ pente RMS 0,165 du relief déclaré) |
| Pas de reprise après verrouillage de la porte | après une perte de texture, un vol restait faux de 19 m jusqu'à la fin | réinitialisation du terrain après 10 trames rejetées cohérentes (écart médian ≤ 10 %) |
| Cap solaire : erreurs d'inclinaison AHRS traitées comme bruit blanc à 10 Hz | filtre trop confiant (6 à 33 % des échantillons dans 2σ), divergence à 150° soleil au zénith | état d'erreur solaire de Gauss-Markov (60 s) : 87 à 99 % dans 2σ ; refus des mesures de σ > 10° |
| Baromètre nominal sous-dispersé | p95 à 600 s : 2,7 m simulé contre 6,0 m mesuré | modèle calé sur Zurich (section 2) |

Résultats (SIMULÉ, après correction, p95 de l'erreur de hauteur sol après coupure, régime baro nominal) :

| Scénario | baro seul | IMU+baro | stéréo seule | fusion (terrain en état) | fusion + DEM (ablation) |
|---|---|---|---|---|---|
| plat, 40 m, bonne texture | 5,2 m | 5,2 m | 2,0 m | 2,2 m | 2,2 m |
| relief, 40 m, bonne texture | 90,6 m | 90,5 m | 15,1 m | 3,4 m | 3,4 m |
| relief, 40 m, perte de texture | 90,6 m | 90,5 m | 743,6 m | 17,6 m | 10,3 m |
| plat, 300 m | 43,7 m | 43,6 m | 53,8 m | 49,7 m | 16,6 m |
| relief, 300 m | 95,1 m | 95,2 m | 64,3 m | 58,1 m | 35,2 m |

Ce que ça veut dire :
- La stéréo (B = 0,20 m, f = 1 000 px) donne la hauteur sol à 40 m, pas à 300 m (0,67 px de disparité).
- La stéréo ne corrige jamais la dérive du baromètre sur l'altitude : avec dérive forte, erreur finale d'altitude 12,6 m pour une dérive vraie de 12,6 m (H4). Seul un DEM lu à la bonne position la réduit (1,7 m).
- Sans aucun GNSS, l'altitude absolue reste inconnue (corrélation 0,99999 avec le décalage baro), mais la hauteur sol est aussi bonne (1,69 m RMSE dans les deux cas).
- Un décalage de disparité de 0,05 px (calibration) donne 22 m de biais de hauteur à 300 m ; le filtre ne le modélise pas. Limite déclarée.
- Hypothèses pré-enregistrées : H1 à H6 confirmées, H7 réfutée (couverture 99,5 % sur plat : filtre un peu trop prudent). Y1 à Y4 confirmées ; Y2 l'est trivialement : au zénith le filtre refuse le soleil et retombe sur le gyro.

Cap (SIMULÉ, Taipei, p95 sur 600 s, biais gyro 0,001 rad/s) : gyro seul 42,1° ; gyro + soleil octobre midi, capteur 0,1°/AHRS 0,5° : 1,70° ; capteur 1°/AHRS 2°, nuages : 6,04° ; 21 juin midi : soleil refusé ; 21 décembre 15 h 30 (élévation 16-18°) : hors champ d'un capteur à ±60°.

## 2. Baromètre réel de Zurich et calage du générateur (MESURÉ)

Commande : `python experiments/s_zurich_vertical.py` (4 s). Sorties : `data/processed/zurich_vertical/`.

Fonction de structure de (baro − photogrammétrie), toutes les paires sur la grille 1 Hz, 45 min, un vol :

| Horizon | 1 s | 10 s | 60 s | 300 s | 600 s | 1 200 s |
|---|---|---|---|---|---|---|
| médiane | 0,22 m | 0,38 m | 0,55 m | 1,53 m | 2,45 m | 4,45 m |
| p95 | 0,76 m | 1,46 m | 1,96 m | 3,98 m | 6,01 m | 8,83 m |

Modèle ajusté (blanc + marche aléatoire + rampe par vol) : **0,30 m ; 0,112 m/√s ; 0,0024 m/s**. Le générateur calé reproduit le p95 mesuré à 60/300/600/1 200 s : 1,91/4,11/6,17/9,63 m contre 1,96/3,98/6,01/8,83 m. L'ancien générateur nominal donnait 1,14/1,67/2,71/4,89 m (trop optimiste au-delà de 60 s) ; l'ancien « fort » 2,2/7,9/15,2/29,6 m. Les deux sont remplacés dans `n_sensor_fusion.py` (nominal = Zurich ; fort = marche 0,15 m/√s + rampe 0,015-0,025 m/s).

Autres mesures :
- Bruit blanc du baro à 10 Hz : 0,16 m. Bruit de la référence photogrammétrique (proxy différence seconde) : 0,03 m.
- **Erreur d'échelle** : les variations d'altitude du baro sont 7,2 % trop grandes par rapport à la photogrammétrie ; le GNSS est d'accord avec la photogrammétrie à 0,3 % près. L'atmosphère standard n'explique que ≈ 1,5 % (INFÉRENCE). Retirer cette échelle ne réduit la dérive à 600 s que de 3,15 à 2,92 m RMS : l'essentiel de la dérive n'est pas l'échelle.
- Aucune dépendance à la vitesse visible (corrélation −0,01 avec V²), mais le vol était à 0,8 m/s : l'effet de la pression dynamique (≈ 138 Pa ≈ 11 m à 15 m/s si entièrement vue par la prise statique) n'est **pas testé**. INFÉRENCE.

Écart restant (pourquoi ce calage ne suffit pas) : un seul vol, un jour, drone captif marché à pied, 460 à 489 m d'altitude, 6 à 10 °C, référence à erreur basse fréquence inconnue (le modèle est donc plutôt pessimiste).

Généralité (PUBLIÉ) :
- Morales et al. 2022 (Matrice 600, 11 vols, altitude GPS/pression contre RTK) : dérive moyenne 0,6 m par 10 min, jusqu'à 1,2 m ([AMT](https://amt.copernicus.org/articles/15/2177/2022/)). Zurich donne 2,45 m médian à 600 s : notre modèle est 2 à 4 fois plus pessimiste, cohérent avec une référence imparfaite.
- Wu et al. 2026 (VTOL thermique, 100/250/500 m, rafales 0-4 m/s) : altitude baro de base contre RTK, RMSE 4,05/1,82/4,76 m ([Sensors](https://pmc.ncbi.nlm.nih.gov/articles/PMC12987359/)).
- Jeux de données avec baro brut et RTK indépendant trouvés : INSANE (MS5611 20 Hz, double RTK, 25-40 m), CTU-MRS MAS et Cooperative UAV (Pixhawk, RTK Emlid). Rien d'utilisable dans NTU VIRAL, MARS-LVIG, UrbanNav, GVINS, EuRoC, UZH-FPV, Blackbird (pas de baro publié). INSANE est testé en section 8.

## 3. Rejeu vertical réel à Zurich : qu'apporte l'IMU ? (MESURÉ)

Même commande. GNSS jusqu'à la coupure, puis baro (± accéléromètre brut 10 Hz tourné par le quaternion PX4), notée contre la photogrammétrie ; 86 coupures toutes les 30 s.

| Horizon | baro seul | baro filtré 1 s | IMU + baro (EKF) | IMU seule après coupure |
|---|---|---|---|---|
| 10 s (médiane) | 0,43 m | 0,33 m | 0,41 m | 2,8 m |
| 60 s | 0,58 m | 0,51 m | 0,51 m | 69 m |
| 300 s | 1,76 m | 1,76 m | 1,68 m | 1 517 m |
| 600 s (p95) | 6,18 m | 5,75 m | 6,33 m | 26 175 m |

Erreur haute fréquence (60 s après coupure, moyenne retirée) : baro 0,44 m, baro filtré 0,38 m, IMU+baro 0,44 m.

Conclusion Zurich : **sur ces données, l'IMU n'apporte rien à la hauteur** ; un simple filtre passe-bas fait mieux. Raison mesurée : l'accéléromètre brut est échantillonné à 10 Hz sans pré-intégration, les vibrations se replient (écart-type vertical 0,62 m/s²). Fuite possible déclarée : le quaternion PX4 peut utiliser le GNSS pour compenser les accélérations (effet du second ordre sur la verticale).

Contre-essai avec une vraie IMU (MESURÉ, INSANE, IMU PX4 à 196 Hz moyennée à 10 Hz, inclinaison par filtre complémentaire gyro + accéléromètre de 2 s, sans vérité ; RTK fixe 1 Hz avant la coupure, RTK plein débit pour noter) : `python experiments/s_insane_vertical.py` (3 s ; script d'un enfant, relancé ici). Médiane de l'erreur de variation d'altitude :

| Séquence (coupures) | horizon | baro brut | baro filtré 1 s | IMU + baro | IMU seule |
|---|---|---|---|---|---|
| mars_2 (25) | 10 s | 0,58 m | 0,34 m | 0,33 m | 0,77 m |
| mars_2 (21 / 15) | 30 s / 60 s | 0,82 / 0,58 m | 0,80 / 0,97 m | 0,62 / 0,55 m | 3,9 / 9,9 m |
| mars_1 (12 / 8) | 10 s / 30 s | 0,26 / 0,36 m | 0,22 / 0,40 m | 0,21 / 0,18 m | 0,95 / 4,2 m |

Bruit haute fréquence (10 s après coupure, moyenne retirée) : mars_2 0,44 (brut) / 0,31 (filtré) / 0,27 m (IMU + baro).

Conclusion : **avec une IMU correctement échantillonnée, l'IMU lisse le baro sans retard** (bruit divisé par ≈ 1,6, et mieux que le passe-bas pendant les montées, à 30-60 s), **mais ne change pas la dérive** (à 60 s, 0,55 m contre 0,58 m pour le baro brut). Seule, elle diverge : 4 m à 30 s, 10 m à 60 s. Échantillon petit (2 vols courts, 12 à 25 coupures qui se chevauchent).

## 4. Hauteur par stéréo : bande d'altitude couverte (SIMULÉ, specs PUBLIÉES)

Commande : `python experiments/s_drift_budget.py` (6 s), partie S. Hypothèses : bruit de disparité 0,1 px par trame, décalage 0,05 px par vol, moyenne sur 10 trames (optimiste : bruits supposés indépendants).

| Caméra | base | f (px) | hauteur max pour 5 % | pour 13 % | disparité à 100 m |
|---|---|---|---|---|---|
| RealSense D435 (375 $, 75 g, 3,4 W) | 50 mm | 686 | 29 m | 75 m | 0,34 px |
| Orbbec Gemini 2 (234 $, 98 g, 2,5 W) | 50 mm | 629 | 27 m | 68 m | 0,31 px |
| OAK-D Lite (269 $, 61 g, 3 W) | 75 mm | 433 | 27 m | 71 m | 0,32 px |
| OAK-D Pro (429 $, 91 g) | 75 mm | 763 | 48 m | 125 m | 0,57 px |
| RealSense D455 (499 $, 116 g, 3,5 W) | 95 mm | 686 | 54 m | 142 m | 0,65 px |
| ZED 2i (499 $, 229 g, 1,9 W) | 120 mm | 774 | 78 m | 204 m | 0,93 px |
| OAK-D LR (779 $, 415 g, 5,5 W) | 150 mm | 736 | 92 m | 241 m | 1,10 px |
| sur mesure 0,30 m (George 2023) | 300 mm | 1 000 | 253 m | > 600 m | 3,0 px |
| sur mesure 0,41 m (Song 2017) | 410 mm | 1 000 | 346 m | > 600 m | 4,1 px |
| sur mesure 1 m (aile) | 1 000 mm | 1 000 | > 600 m | > 600 m | 10 px |

Sources specs (PUBLIÉ, pages constructeur, f calculé depuis le champ horizontal) : [D435](https://www.realsenseai.com/products/stereo-depth-camera-d435/), [D455](https://www.realsenseai.com/products/real-sense-depth-camera-d455f/), [OAK-D Lite](https://docs.luxonis.com/hardware/products/OAK-D%20Lite), [OAK-D Pro](https://docs.luxonis.com/hardware/products/OAK-D%20Pro), [OAK-D LR](https://docs.luxonis.com/hardware/products/OAK-D%20LR) (objectif 82° supposé), [ZED 2i](https://docs.stereolabs.com/docs/products/cameras/zed/specifications), [Gemini 2](https://www.orbbec.com/products/stereo-vision-camera/gemini-2/). Bruit sous-pixel : < 0,1 px RMS sur cible plane texturée selon le [guide RealSense](https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance/) ; ce n'est pas une garantie en extérieur. Les précisions constructeur (< 2 % à 2-4 m) ne disent rien de la hauteur à 50-300 m.

Résultats de vol publiés : George et al. 2023 (base 0,30 m, 40 à 100 m) : erreur de trajectoire 2,2 m au mieux à 60 m, mais 102 à 123 m à 100 m en VO stéréo pure et 9,5 à 12,9 m avec IMU ([PDF](https://mdpi-res.com/d_attachment/drones/drones-07-00036/article_deploy/drones-07-00036-with-cover.pdf?version=1705552179)). Cela confirme la chute rapide au-delà de quelques dizaines de mètres par décimètre de base. PUBLIÉ.

Comparaison : baro (Zurich) + DEM sur terrain plat donne 4,2 % à 100 m et 2,8 % à 150 m après 600 s ; sur relief à 15 % de pente avec 100 m d'erreur de position, 15,6 % à 100 m.

Stéréo temporelle multi-vues (Song et al. 2017, PUBLIÉ https://pmc.ncbi.nlm.nih.gov/articles/PMC5298584/) : la base vaut V·Δt, donc l'erreur relative de hauteur ne peut pas être meilleure que l'erreur relative de vitesse. Or c'est la vitesse qu'on cherche (flux optique = V/h). **Cercle fermé** : la stéréo temporelle n'apporte pas d'échelle sans source de vitesse indépendante (GNSS dans Song 2017). INFÉRENCE (géométrie).

Ce que la hauteur achète :
- vitesse caméra : erreur d'échelle = erreur relative de hauteur ;
- recalage carte : le zoom à chercher couvre ± 3σ de l'erreur relative ; à 5 % un seul zoom suffit presque, à 13 % il en faut plusieurs (ALTO : 7 % de zoom faisait tomber ZNCC brut à 37-40 %, MESURÉ par la piste A).

## 5. Altitude tirée de la vidéo : biais d'évaluation (SIMULÉ)

Partie V du même script. Terrain plat, 120 m sol, 15 m/s, dérive le long de la route due à la seule hauteur :

| Source de hauteur | 60 s (900 m) p50/p95 | 300 s (4,5 km) | 600 s (9 km) |
|---|---|---|---|
| baro calé Zurich + 3 m d'erreur à la coupure | 16/44 m | 82/232 m | 183/520 m |
| baro calé Zurich, hauteur parfaite à la coupure | 3/9 m | 31/90 m | 87/248 m |
| altitude « vidéo » (type SfM, 1 % d'échelle, sans dérive) | 6/18 m | 30/87 m | 61/174 m |

Lecture : une altitude reconstruite depuis la vidéo **sous-estime la dérive d'un facteur 2,7 à 3** par rapport à un vrai baromètre. Elle n'a pas la marche aléatoire du baro et partage l'information de la caméra (fuite).

Protocole honnête proposé :
1. Ne jamais appeler « baromètre » une altitude tirée de la vidéo.
2. Baro synthétique = altitude de référence (GNSS/RTK du vol ou de la simulation) + erreur tirée du modèle calé sur des journaux réels (Zurich : 0,30 m ; 0,112 m/√s ; 0,0024 m/s), au moins 20 tirages, résultats en médiane/p95.
3. Montrer en parallèle le résultat avec un vrai baro (Zurich ou journaux PX4) pour vérifier le calage.
4. Déclarer l'erreur à la coupure (GNSS vertical + DEM) : c'est le terme dominant à 10 min.

## 6. Capteur solaire (SIMULÉ + PUBLIÉ)

σ_cap = √((σ_capteur/cos é)² + (σ_inclinaison · tan é)²). Partie H du script. Extraits (degrés) :

| capteur / inclinaison AHRS | é = 30° | 50° | 70° | 80° |
|---|---|---|---|---|
| 0,1° / 0,5° | 0,31 | 0,62 | 1,40 | 2,89 |
| 0,5° / 0,5° | 0,65 | 0,98 | 2,01 | 4,04 |
| 1,0° / 2,0° | 1,63 | 2,85 | 6,22 | 12,72 |

C'est l'erreur d'inclinaison de la centrale, pas le capteur, qui domine dès 40° d'élévation.

Géométrie Taïwan (éphémérides NOAA, 7 h-17 h, capteur à ±60°, σ ≤ 2°) : 50 à 74 % du jour avec bonne centrale ; 11 à 21 % avec centrale « terrain » (2°). Élévation à midi : 43° en décembre (Taipei) à 88-89° en juin-juillet.

Ensoleillement CWA (PUBLIÉ, normales 1991-2020, https://www.cwa.gov.tw/V8/C/C/Statistics/MonthlyMean/MOD/Taiwan_sunshine.html ; `python experiments/s_taiwan_sunshine.py`) : fraction d'ensoleillement annuelle Taipei 0,32, Taichung 0,46, Kaohsiung 0,52 ; Taipei 0,23 à 0,25 de janvier à avril ; Taichung 0,58 en octobre. Produit (INFÉRENCE, indépendance supposée) : soleil exploitable 40 % du jour à Taichung en octobre, 12 % à Taipei en janvier.

Part de la dérive transversale d'ALTO (198 m, 3°) retirée, SIMULÉ sur 4,6 km : bonne centrale 76 % (161 → 38 m), avec 40 % de disponibilité et gyro entre deux 75 % (40 m), centrale à 2° 30 % (113 m). Sur le vrai mouvement caméra d'ALTO (section 7) : un capteur à ≈ 1° fait passer le travers de 196 m à 31 m (84 %), à 2,85° à 32 m ; l'erreur finale ne baisse que de 6 % car l'erreur le long de la route domine. Le capteur solaire ne touche pas aux 575 m le long de la route.

Matériel (PUBLIÉ) :

| Capteur | précision | champ | masse / puissance | prix |
|---|---|---|---|---|
| [Foresail-1 PSS](https://github.com/foresail/fs1_psd_sun_sensor), libre, photodiode 4 électrodes + trou | < 5° (article) ; ± 1° (README) | ± 50° | 4 g / 4 mW | ≈ 100 € de pièces |
| [Foresail-1 DSS](https://github.com/foresail/fs1_dss_sun_sensor), libre, CMOS + trou lithographié | < 0,5° | ± 18° | 4,4 g / 10 mW moyen | ≈ 200 € |
| [Solar MEMS nanoSSOC-A60](https://solar-mems.com/wp-content/uploads/2024/01/nanoSSOC-A60.pdf) | < 0,5° (3σ) | ± 60° | 4 g / < 10 mW | 2 500 € |
| [Bradford Mini-FSS](https://www.bradford-space.com/products/mfss), quadrant passif | ± 1,5° sans table, ± 0,2° avec | ± 64° | 50 g / 0 W | non publié |
| Compas à polarisation (caméra Sony IMX250MZR, [Pan et al.](https://doi.org/10.1364/OE.510283)) | 0,10° ciel clair, 0,18° voile, 0,3-0,5° nuages épais (sol) | ciel | caméra | 1 500-3 000 $ |

Essais en vol publiés : capteur solaire à photorésistances sur AR.Drone, incertitude « surtout sous 10° » ([Liu et al. 2013](https://www.uaslaboratory.com/_files/ugd/49bf50_d749c969f91a4d4cabaf281f1edf2454.pdf)) ; compas à polarisation sur quadrirotor < 2° ([Zhi et al. 2018](https://pmc.ncbi.nlm.nih.gov/articles/PMC5795797/)) ; ≈ 0,5° à 310 m ([Zhao et al. 2022](https://doi.org/10.1016/j.measurement.2022.110734)). Le couplage inclinaison → cap mesuré par JPL sur rover : ≈ 0,5° de cap par degré de roulis/tangage ([Trebi-Ollennu et al.](https://robotics.jpl.nasa.gov/media/documents/IEEETRA_Sun.pdf)) ; notre formule donne ce rapport pour une élévation de 27° (élévation de l'essai JPL non vérifiée).

Lecture pour Taïwan (INFÉRENCE) : un compas à polarisation tolère le voile nuageux mieux qu'un capteur solaire direct (l'ensoleillement CWA ne compte que le soleil direct). C'est la seule piste « cap céleste » qui ne meurt pas sous un ciel de Taipei à 25 % d'ensoleillement, mais il coûte 1 500 $ et plus, et garde le même couplage avec l'inclinaison.

Alternative testée par un autre agent (MESURÉ, `experiments/u1_shadow_compass.py`, `data/processed/u1_shadow_compass/explicit_shadow_test.json`) : lire le cap dans les ombres de l'image caméra. Sur ALTO, 6 images valides sur 100 et écart-type circulaire de 51° ; sur Wufeng, 1 patch valide sur 30. Abandonné (seuil d'arrêt : 10°).

## 7. Combinaison : quel capteur par dollar et par watt (SIMULÉ, puis contrôle sur le vrai trajet ALTO)

Partie C : 1 000 vols, 4,6 km à 15 m/s, croisière à altitude constante à 120 m au-dessus d'un relief roulant (sol 60 à 181 m sous le drone). L'échelle figée reproduit l'ordre de grandeur d'ALTO (17 % d'erreur relative à la fin, 484 m le long de la route ; ALTO : 13 %, 575 m).

| Échelle | Cap | erreur finale p50 / p95 | distance avant p95 > 60 m | coût ajouté | puissance |
|---|---|---|---|---|---|
| figée (actuel) | caméra 3° | 549 / 1 360 m | 315 m | 0 $ | 0 W |
| baro seul | caméra 3° | 542 / 1 329 m | 315 m | 0 $ | 0 W |
| baro + DEM | caméra 3° | 234 / 490 m | 563 m | 0 $ | 0 W |
| baro + DEM | gyro | 147 / 320 m | 900 m | 0 $ | 0 W |
| baro + DEM | soleil, centrale 2° | 187 / 382 m | 720 m | ≈ 150 $ | 0,01 W |
| baro + DEM | soleil, bonne centrale | 123 / 292 m | 945 m | ≈ 1 650 $ | 1 W |
| OAK-D LR | gyro | 205 / 506 m | 540 m | 779 $ | 5,5 W |
| stéréo 1 m | soleil, bonne centrale | 48 / 110 m | 2 520 m | ≈ 2 050 $ | 5 W |

Le baro seul n'apporte rien en croisière à altitude constante : il suit l'altitude, pas le sol. Il faut le DEM. Coûts : capteur solaire libre type Foresail ≈ 150 $ de pièces (tableau section 6) ; « bonne centrale » (inclinaison à 0,5° en vol sans GNSS) ≈ 1 500 $, INFÉRENCE non sourcée ; stéréo 1 m sur mesure ≈ 400 $ (deux modules OV9282 à 90 $ + barre + calcul), INFÉRENCE.

Sensibilité (SIMULÉ, même graine, erreur finale le long de la route, médiane) :

| Variante | échelle figée | baro + DEM | OAK-D LR | stéréo 1 m |
|---|---|---|---|---|
| relief de référence (sol 60-181 m sous le drone) | 484 m | 103 m | 169 m | 19 m |
| plaine (relief × 0,1 ; type Wufeng) | 87 m | 100 m | 173 m | 19 m |
| relief × 2 (quelques vols frôlent le sol) | 984 m | 121 m | 162 m | 18 m |
| erreur DEM 8 m (DSM en ville ou forêt) | 484 m | 247 m | 169 m | 19 m |

Commandes : `python experiments/s_drift_budget.py --terrain-scale 0.1 --out data/processed/drift_budget_flat`, `--terrain-scale 2 --out data/processed/drift_budget_hilly2x`, `--dem-sigma 8 --out data/processed/drift_budget_dem8m`.

Lecture : **en plaine, le DEM n'apporte rien** (l'erreur de hauteur à la coupure domine) ; le gain gratuit vient alors du gyro. Le DEM devient décisif dès que le sol varie de plus de quelques pourcents de la hauteur de vol. Avec un DEM faux de 8 m, une OAK-D LR repasse devant.

Contrôle sur le vrai trajet ALTO (MESURÉ pour la caméra et le terrain, baro SIMULÉ ; script d'un autre agent, non modifié : `.venv/bin/python experiments/u2_baro_dem_scale.py`, résultats `data/processed/u2_baro_dem_scale/summary.json`) : recalage caméra sur les 300 premiers mètres, puis 4,3 km sans recalage.

| Hauteur utilisée pour l'échelle | erreur d'échelle médiane | erreur médiane | erreur finale |
|---|---|---|---|
| figée (équipe) | 9,1 % | 472 m | 608 m |
| baro simulé − DEM lu à la position estimée | 10,5 % | 372 m | 639 m |
| hauteur sol vraie (oracle) | 0 % | 270 m | 426 m |

Lecture : **ma simulation surestime le gain du DEM sur ALTO.** Elle attribue toute l'erreur le long de la route à la hauteur sol ; sur le vrai vol, la hauteur parfaite ne retire que 30 % de l'erreur finale. Le reste vient du cap (198 m en travers) et d'erreurs de la vitesse caméra elle-même (calibration flux → sol, inclinaison, parallaxe du relief). INFÉRENCE sur la répartition. Le baro + DEM échoue sans recalage parce que la position estimée dérive.

Recalages simulés sur le vrai trajet ALTO (caméra MESURÉE, baro et recalages SIMULÉS : recalage = vérité + 15 m, zoom renouvelé à 5,7 % près ; 20 graines) : `python experiments/s_alto_fix_spacing.py` (1 s ; script d'un enfant, reproduit u2 à 0,001 m près). Part des recalages arrivant avec plus de 60 m d'erreur (rayon de recherche du banc carte) :

| Espacement | figée (équipe) | baro − DEM | hauteur vraie (oracle) |
|---|---|---|---|
| 300 m | 31,8 % | 24,6 % | 18,9 % |
| 500 m | 71,9 % | 58,8 % | 39,4 % |
| 1 000 m | 98,8 % | 100 % | 100 % |

Lecture : **avec des recalages, le baro + DEM sert** : il retire environ un quart des recalages ratés à 300 m (31,8 → 24,6 %) et un cinquième à 500 m. Il ne permet pas d'espacer les recalages au-delà de ≈ 300 m sur ALTO, même avec une hauteur parfaite : l'erreur restante vient d'ailleurs (cap, vitesse caméra). Cela retombe sur la « falaise » de 300 à 400 m mesurée par l'équipe.

Cap sur le vrai trajet ALTO (caméra MESURÉE ; cap oracle = lacet ALTO ; capteurs SIMULÉS sur 20 graines) : `python experiments/s_alto_heading.py` (3 s ; script d'un enfant, reproduit u2 à 0,04 m près). Erreur finale et composantes (projetées sur la direction de vol des 200 derniers mètres) :

| Échelle | Cap | erreur finale | le long | en travers |
|---|---|---|---|---|
| figée | figé (équipe) | 608 m | −576 m | −196 m |
| figée | oracle | 572 m | −571 m | +31 m |
| figée | soleil ≈ 1° | 573 m | −571 m | +31 m |
| figée | soleil 2,85° | 583 m | −573 m | +32 m |
| hauteur vraie | figé | 426 m | −372 m | −208 m |
| hauteur vraie | oracle | 369 m | −367 m | +33 m |
| hauteur vraie | soleil ≈ 1° | 371 m | −367 m | +34 m |

Lecture : le cap compte en quadrature avec l'erreur le long de la route, donc il ne pèse que lorsque celle-ci est réduite. La variante gyro (637 m) n'est pas interprétable : elle suppose une ligne ALTO par seconde, base de temps non vérifiée.

D'où viennent les 367 m restants (MESURÉ, diagnostic avec oracles ; `python experiments/s_alto_speed_residual.py`, 1 s, script d'un enfant reproduisant 368,5 m) :

| Variante (hauteur vraie + cap oracle) | erreur finale | le long | rapport médian pas estimé / pas vrai |
|---|---|---|---|
| calibration flux → sol sur 300 m (équipe) | 369 m | −367 m | 0,90 |
| calibration sur tout le trajet (diagnostic, vérité utilisée) | 74 m | −73 m | 0,97 |
| calibration isotrope (échelle + rotation) sur 300 m | 439 m | −418 m | 0,90 |

Sur 22 tronçons de 200 m, le rapport des pas suit d'abord l'amplitude du flux en pixels (r = 0,56), puis la hauteur sol (0,35) ; l'inclinaison (−0,29) et la pente du terrain (0,12-0,16) pèsent peu. Diagnostic : la calibration apprise sur 300 m est biaisée de ≈ 7-10 % et le biais dépend de l'amplitude du mouvement image (comportement non linéaire de l'estimateur de flux, flou de bougé ; hypothèse non prouvée). Pistes : calibrer sur plus de distance GNSS ou à plusieurs vitesses, corriger la non-linéarité du flux, et laisser chaque recalage carte renouveler l'échelle (ce que l'équipe fait déjà). C'est le plus gros gisement, devant tout capteur.

## 8. Autres baromètres réels contre RTK : INSANE et journaux PX4 (MESURÉ)

Commande : `python experiments/s_insane_baro.py` (1 s ; script écrit par un enfant, relu et relancé ici). Données : [INSANE](https://www.aau.at/en/smart-systems-technologies/control-of-networked-systems/datasets/insane-dataset/) (BSD-2 + interdiction de vente), multirotor 3 kg, baro MS5611 20 Hz, RTK u-blox, seuls les échantillons RTK fixes sont gardés. Trois séquences de 18, 8 et 26 Mo dans `data/raw/insane/`.

| Séquence | durée RTK fixe | altitude | 10 s médiane / p95 | 60 s | 120 s | erreur d'échelle |
|---|---|---|---|---|---|---|
| mars_1 | 100 s | 16-21 m | 0,38 / 1,40 m | 0,66 / 1,64 m | – | +6,7 % |
| mars_2 | 164 s | 16-32 m | 0,48 / 2,06 m | 0,63 / 2,23 m | 0,96 / 2,48 m | +3,0 % |
| outdoor_1 | 219 s (22 % fixe) | 0-24 m | 0,09 / 0,29 m | 0,63 m (64 paires) | – | −0,8 % |
| Zurich (rappel) | 2 713 s | 460-489 m | 0,38 / 1,46 m | 0,55 / 1,96 m | 0,80 / 2,55 m | +7,2 % |

Lecture :
- Aux horizons courts (10 à 120 s), deux engins, deux baromètres, deux références différentes (photogrammétrie, RTK) donnent le **même ordre de grandeur** : ≈ 0,4 m à 10 s, ≈ 0,6 m à 60 s, ≈ 0,8-1 m à 120 s. Le modèle calé sur Zurich est plausible jusqu'à 2 min.
- **Au-delà de 2 min, rien ne le confirme** : les séquences INSANE sont trop courtes. La dérive à 5-10 min reste mesurée sur un seul vol (Zurich) ; la littérature (Morales 2022 : 0,6 m/10 min) la donne plus faible.
- L'**erreur d'échelle** positive revient (+3 à +7 %) sur deux engins sur trois : le baro exagère les variations d'altitude. Cause non identifiée (souffle des hélices, température). Conséquence pratique : estimer un facteur d'échelle baro avant la coupure si l'altitude varie pendant la phase GNSS. INFÉRENCE.

Journaux PX4 Flight Review avec RTK fixe (MESURÉ, CC BY 4.0, exportés par la piste C) : `python experiments/s_px4_rtk_baro.py` (2 s ; script d'un enfant, relancé ici). Référence = altitude GNSS RTK fixe du même journal (indépendante du baro, mais pas un relevé géodésique).

| Journal | en vol | altitude / vitesse | médiane 60 s | 300 s | 600 s | 1 200 s | p95 600 s |
|---|---|---|---|---|---|---|---|
| 036fb3a7, octo 10" | 1 167 s | 3-108 m ; 7,9 m/s médiane, 10,3 max | 1,08 m | 1,34 m | 1,13 m | – | 4,48 m |
| a2a30b98, quadri | 1 951 s | 3-15 m ; 0,7 m/s | 0,73 m | 0,81 m | 0,99 m | 1,45 m | 2,77 m |
| 53736001, quadri (firmware non standard) | 328 s en 3 morceaux | −6-96 m ; 0,3 m/s | 0,61 m | (95 paires) | – | – | – |
| Zurich (rappel) | 2 713 s | 460-489 m ; 0,8 m/s | 0,55 m | 1,53 m | 2,45 m | 4,45 m | 6,01 m |

Lecture :
- À 5-20 min, les deux journaux PX4 longs dérivent **deux fois moins** que Zurich (médiane ≈ 1 m à 600 s contre 2,45 m ; p95 2,8-4,5 m contre 6,0 m). Avec Morales 2022 (0,6 m/10 min), le modèle de Zurich est une **borne pessimiste** ; c'est lui qu'on garde dans les simulations, faute de mieux et par prudence.
- Aux horizons courts, le bruit dépend de l'engin : 0,5 à 0,8 m médian à 10 s sur les PX4 (souffle des hélices, vol à 3-15 m), 0,4 m à Zurich et INSANE.
- **Effet de la vitesse** (journal octo, jusqu'à 10 m/s) : coefficient 0,0066 m par (m/s)², soit ≈ 0,7 m à 10 m/s et ≈ 1,5 m à 15 m/s par extrapolation. Petit devant la dérive, mais confondu avec l'altitude dans la régression. Les deux autres journaux sont trop lents pour conclure (leurs coefficients, de signes opposés, sont inutilisables).
- Erreurs d'échelle : −5,9 % (octo), −17 % (a2a30b98, sur 12 m de dénivelé seulement), +5,2 %. Avec Zurich (+7,2 %) et INSANE (+3,0 % et +6,7 %), **l'échelle baro se trompe de 3 à 7 % dans un sens ou dans l'autre** : il faut l'estimer avant la coupure dès que l'altitude varie.

## 9. Limites et questions ouvertes

- Les sections 4 à 6 et la partie synthétique de la section 7 sont SIMULÉES. Sont MESURÉS : bruit baro (Zurich 45 min, INSANE 3 vols courts, 3 journaux PX4 RTK), rejeux IMU (Zurich, INSANE), et le mouvement caméra ALTO dans les rejeux de la section 7 (baro, recalages et capteurs de cap y restent simulés). Aucun vol réel mesuré ici n'est à 100-300 m au-dessus du sol à 15 m/s pendant 10 min.
- La simulation synthétique de la section 7 surestime le gain du DEM : elle met toute l'erreur le long de la route sur la hauteur sol, alors que sur ALTO la hauteur n'en explique qu'environ un tiers.
- Le DEM GLO-30 est un modèle de surface (toits, arbres) : c'est ce que voit la caméra, mais le DEM et le sol vrai peuvent différer de 10 m en ville ou en forêt. INFÉRENCE.
- L'effet de la vitesse sur la prise statique du baro n'est mesuré que jusqu'à 10 m/s (≈ 0,7 m), sur un seul journal.
- Biais gyro résiduel de 0,005 °/s : hypothèse non mesurée sur le matériel de l'équipe.
- Questions pour l'équipe (dans `questions.md`) : date/heure de la démo, erreur d'inclinaison de l'autopilote, relief sous la trajectoire.

## Fichiers

- `experiments/n_sensor_fusion.py` (revu, corrigé) → `data/processed/sensor_fusion/`
- `experiments/s_zurich_vertical.py` → `data/processed/zurich_vertical/`
- `experiments/s_drift_budget.py` → `data/processed/drift_budget/`
- `experiments/s_taiwan_sunshine.py` → `data/processed/taiwan_sunshine/` (brut : `data/raw/cwa_sunshine/`)
- `experiments/s_insane_baro.py` → `data/processed/insane_baro/` (brut : `data/raw/insane/`)
- `experiments/s_insane_vertical.py` → `data/processed/insane_vertical/`
- `experiments/s_px4_rtk_baro.py` → `data/processed/px4_rtk_baro/` (journaux exportés par la piste C)
- `experiments/s_alto_fix_spacing.py`, `experiments/s_alto_heading.py` → `data/processed/alto_fix_spacing/`, `data/processed/alto_heading/` (importent `experiments/u2_baro_dem_scale.py` d'un autre agent, sans le modifier)
- `experiments/s_alto_speed_residual.py` → `data/processed/alto_speed_residual/`

## 10. Calibration en ligne de l'odométrie caméra par les recalages (pré-enregistré avant exécution)

Question : la calibration figée sur 300 m explique l'essentiel de l'erreur ALTO (369 → 74 m avec une calibration parfaite, section 7). Peut-on apprendre en vol, sans vérité, l'échelle et le biais de cap de l'odométrie caméra à partir des déplacements entre recalages carte successifs acceptés ?

Méthode (`experiments/s_alto_online_calib.py`) : logique de recalage de l'équipe reprise de `h_alto_end_to_end.py` (corrélation de luminosité, seuil de score 0,33, recherche dimensionnée par l'incertitude). État : position, log-échelle, biais de cap. Entre deux recalages acceptés, on compare le déplacement recalage → recalage au déplacement caméra intégré : le rapport des longueurs donne la log-échelle, la différence d'angle le cap, avec un bruit de √2 × 15 m / distance. La vérité ne sert qu'à noter.

Critères fixés avant de lancer :
- **P1** : recalages tous les 1 000 m, erreur médiane ≤ 40 m (équipe : 56 m) **et** aucun recalage accepté faux de plus de 50 m.
- **P2** : recalages tous les 300 m, erreur médiane pas pire que celle de l'équipe (31 m) de plus de 3 m.
- **P3** : recalages tous les 300 m jusqu'à 2 km après la coupure, puis plus rien : la distance parcourue avant 100 m d'erreur est au moins 1,5 fois celle de l'odométrie non calibrée.
- Échec de sécurité : un recalage faux de plus de 50 m accepté dans une configuration où l'équipe n'en avait aucun.

Résultats (MESURÉ : images ALTO et vrais recalages par corrélation ; `python experiments/s_alto_online_calib.py`, 11 min ; script d'un enfant, la variante équipe reproduit `findings.md` à 0,5 m près). Erreur de position médiane / finale, recalages acceptés / rejetés / acceptés mais faux de plus de 50 m :

| Espacement | équipe (zoom renouvelé) | en ligne (recalage → recalage) | en ligne + zoom |
|---|---|---|---|
| 300 m | 31 / 38 m ; 12/1/0 | 36 / 360 m ; 10/5/**1** | **30** / 116 m ; 12/2/0 |
| 500 m | 36 / 36 m ; 6/1/0 | 162 / 81 m ; 3/4/0 | **30** / 87 m ; 7/1/0 |
| 1 000 m | **56** / 10 m ; 4/0/0 | 162 / 164 m ; 2/1/0 | 63 / 138 m ; 4/0/0 |
| 2 000 m | 115 / 197 m ; 1/0/0 | 107 / 96 m ; 1/0/0 | 108 / 174 m ; 1/0/0 |

Coupure des recalages à 2 km (recalages tous les 300 m avant) : distance avant 50 m / 100 m d'erreur : équipe 200 / 952 m ; en ligne + zoom 631 / 982 m ; en ligne 476 / 902 m. L'odométrie sans aucun recalage dépasse déjà 100 m au moment de la coupure.

Paramètres appris en vol (en ligne + zoom, 300 m) : log-échelle +0,15 (pas allongés de 16 %), biais de cap +3,2°. Ils vont dans le sens des défauts mesurés par ailleurs (pas 10 % trop courts, cap faux de 3°). L'oracle « calibration sur tout le trajet » ne se ramène pas à une échelle + rotation (valeurs singulières relatives 3,1 et 1,1) : la matrice A0 apprise sur 300 m de ligne droite est mal conditionnée dans l'axe transversal.

Verdict pré-enregistré :
- **P1 échoue** : à 1 000 m, 63 m (en ligne + zoom) et 162 m (en ligne) contre ≤ 40 m visés ; l'équipe fait 56 m.
- **P2** : réussi pour en ligne + zoom (30 m contre 31 m) ; échoue pour en ligne seul (36 m).
- **P3 échoue** : mon critère disait « odométrie non calibrée », formulation ambiguë. Sans aucun recalage, l'erreur dépasse déjà 100 m à la coupure (rapport indéfini). Contre l'équipe (recalages, échelle par le zoom, sans calibration en ligne) : 982 m contre 952 m avant 100 m d'erreur (× 1,03). Seul le seuil de 50 m est franchi plus tard (631 m contre 200 m, × 3,2), critère non pré-enregistré.
- **Échec de sécurité** pour la variante en ligne seule : un recalage faux accepté à 300 m (l'équipe : aucun).

Lecture : les paramètres appris sont justes en direction, mais un couple de recalages à 15 m près sur 300 m mesure l'échelle à ≈ 7 % près, autant que le défaut à corriger. Les premières mises à jour bruitées déplacent la prédiction, la recherche part du mauvais endroit et des recalages sont rejetés ou faux. Le zoom du recalage reste un meilleur capteur d'échelle que le déplacement recalage → recalage. Piste suivante (non testée) : n'apprendre que le biais de cap (stable : +3,2 à +4,2° sur tous les espacements) en gardant le zoom de l'équipe pour l'échelle, et réapprendre A0 comme une similitude sur une trajectoire avec virages.

## Prochaines expériences

1. Calibration de la vitesse caméra : apprendre flux → sol sur plus de distance GNSS (500-1 000 m) ou à plusieurs amplitudes de flux, et mesurer la non-linéarité de l'estimateur de flux (r = 0,56 avec l'amplitude) ; potentiel mesuré : 369 → 74 m avec une calibration parfaite. À faire avec la piste caméra.
2. Effet de la vitesse sur la prise statique du baro sur des vols rapides (> 15 m/s) avec RTK : un seul journal va jusqu'à 10 m/s.
3. Mesurer l'erreur d'inclinaison réelle de l'autopilote de l'équipe sans GNSS (vol filmé + journal) : c'est elle qui décide si un capteur solaire sert à quelque chose.
4. Ajouter au filtre un état de facteur d'échelle baro estimé avant la coupure (erreurs d'échelle de 3 à 7 % mesurées, dans les deux sens).
5. Retrouver la base de temps d'ALTO (lignes par seconde) pour tester la variante gyro sur le vrai trajet.
