# Piste C : données, rejeu et simulation (rapport de nuit, 2026-10-03)

Étiquettes : MEASURED (exécuté sur données réelles), SIMULATED (générateur déclaré), PUBLISHED (source citée), INFERENCE. Inventaire détaillé : `docs/research/data-manifest.md`.

## Recommandation : la démo honnête la plus solide

1. **Preuve chiffrée sur vol réel : ALTO Round 2 Train** (MEASURED). Ce vol réel de 37,4 km, horodaté, n'a jamais servi au réglage. Commande : `AltoConfig(data_root="data/raw/alto/round2", section="Train")`.
   - **La réplication avec paramètres gelés (section suivante) montre que les 26–31 m de Val ne se transfèrent pas** : médiane de 94–177 m selon l'espacement des recalages, avec de fortes variations d'une section à l'autre.
   - Pour la démo : montrer la variance (3 sections bonnes sur 8) et le mécanisme, pas le meilleur chiffre de Val.
2. **Histoire visuelle à Taïwan : `wufeng_sim_base`** (SIMULATED, imagerie réelle). La caméra est rendue depuis l'orthophoto 2020 et la carte est l'orthophoto 2018. L'IMU, le baro et le GNSS sont synthétiques, avec leurs bruits déclarés. C'est la seule séquence nadir sur Taïwan avec carte d'une autre année. À l'écran, l'étiqueter « capteurs simulés, images réelles ».
3. **Hauteur sur données réelles** : baro brut avec vérité indépendante. Zurich fournit une vérité photogrammétrique, INSANE une vérité RTK, PX4 deux journaux RTK. Les mesures elles-mêmes reviennent à la piste B.
4. **Gazebo** : seulement pour un plan « le système tourne en boucle avec coupure GNSS », avec le correctif ci-dessous. Il n'apporte pas de preuve chiffrée, car le terrain est procédural.
5. **À éviter** :
   - Zurich pour la localisation sur carte : sa caméra regarde vers l'avant, à hauteur de rue.
   - La 2ᵉ vidéo PX4 (`aa0ae4df`) : le décalage vidéo/journal y est ambigu.
   - Les vidéos YouTube dans la vidéo du jury, tant que leur licence n'est pas connue.

## Réplication tenue à l'écart : ALTO Round 2 Train (résultat négatif, MEASURED)

- **Protocole pré-enregistré avant tout calcul** : `data/processed/t_alto_heldout/preregistration.md`. Tous les paramètres de `h_alto_end_to_end.py` sont gelés tels que réglés sur Val. Script : `experiments/t_alto_heldout.py`.
- **Fidélité** : le code refactoré reproduit les 14 lignes du tableau de findings 3.4 sur Val à moins de 1 m près, avec les mêmes comptes de recalages (`val_reproduction.csv`).
- **Fuite** : aucune. Le point du Round 2 Train le plus proche de Val est à 37,75 km.
- **Données** : 8 sections d'environ 4,6 km et un vol complet de 37,4 km. Les nombres ci-dessous ont été relus dans `results.csv`.

  | Configuration | Val (findings) | Médiane des 8 sections | Min–max |
  |---|---|---|---|
  | caméra seule | 472 m | 219 m | 68–420 m |
  | recalage tous les 100 m, 7 voisines | 26 m | 177 m | 17–725 m |
  | 300 m, 7 voisines | 31 m | 94 m | 20–339 m |
  | 300 m, recherche dimensionnée + seuil | 31 m | 128 m | 26–1142 m |
  | 1000 m, recherche dimensionnée + seuil | 56 m | 160 m | 50–980 m |
  | 2000 m, recherche dimensionnée + seuil | 116 m | 191 m | 51–401 m |

- **Vol complet de 37 km** (médiane / fin ; recalages utilisés / rejetés / faux de plus de 50 m) :
  - sans recalage : 1120 / 2392 m ;
  - recalage tous les 300 m : 1091 / 6421 m (43 / 56 / 2) ;
  - tous les 1000 m : 1490 / 7240 m (14 / 15 / 2) ;
  - tous les 2000 m : 593 / 751 m (7 / 10 / 1).
- **Prédictions** :
  - P1 fausse : médiane sous 60 m dans seulement 1, 3 et 3 sections sur 8 pour 100, 200 et 300 m.
  - P2 fausse : aucun recalage faux dans seulement 6 sections sur 8.
  - P3 fausse.
  - P4 vraie : le seuil 0,33 rejette davantage de recalages corrects.
- **Conséquence (règle pré-enregistrée)** : les chiffres de Val (26–31 m) doivent être présentés comme **réglés sur le jeu de test**. Ils se reproduisent dans 3 sections sur 8 (1, 4 et 8 : 17–33 m) et échouent ailleurs.
- **Mécanisme probable (INFERENCE, à tester)** : la calibration avant brouillage, faite avec 3 recalages et une grille de zoom de 0,60 à 1,00, est instable.
  - Zoom calibré : 1,00 ; 0,65 ; 0,60 ; 0,95 ; 0,80 ; 0,65 ; 0,65 ; 1,00. Trois sections butent sur un bord de la grille.
  - Le zoom n'est pas monotone avec l'altitude : 1,00 et 0,65 tous deux à 528 m.
  - Décalages calibrés jusqu'à 37 m, contre 7,6 m sur Val.
  - Les scores des recalages acceptés sont plus bas (médiane par section 0,19–0,55).
  - L'altitude est plus haute que sur Val : 437–548 m au-dessus de l'ellipsoïde contre 432 m.
- Fichiers : `data/processed/t_alto_heldout/{results,summary,calibration,fix_scores,leak_check}.csv` et `sections.png`.
- Le Round 2 n'a pas servi à régler quoi que ce soit.

### Diagnostic de l'échec (MEASURED, la vérité sert uniquement à l'évaluation)

Script : `experiments/t_alto_diag.py`. Sorties : `data/processed/t_alto_heldout/diag/`.

**Méthode.** Recalages pris seuls tous les 300 m, avec une recherche centrée sur la vraie position. On compare deux zooms : le zoom calibré par la chaîne, et le zoom qui maximise le score sur une grille large de 0,40 à 1,40.

**Part des recalages à moins de 30 m de la vérité :**

| Section | Zoom calibré | Zoom au meilleur score |
|---|---|---|
| Val | 71 % | – |
| S1 | 86 % | 86 % |
| S2 | 43 % | 100 % |
| S3 | 21 % | 93 % |
| S4 | 57 % | 7 % (zoom 0,40) |
| S5 | 86 % | 21 % (zoom 0,40) |
| S6 | 21 % | 14 % (zoom 0,40) |
| S7 | 43 % | 86 % |
| S8 | 94 % | 69 % |

**Ce que j'en conclus** (je ne reprends pas le verdict « terrain » du sous-agent) :
1. **La calibration du zoom est le premier mécanisme.** Avec un meilleur zoom, S2, S3 et S7 passent de 21–43 % à 86–100 % de recalages justes. La calibration à 3 recalages choisit un mauvais zoom.
2. **Le score n'est pas un oracle fiable.** En S4–S6, le meilleur score tombe au bord inférieur de la grille (0,40), où les recalages sont faux (86 m). Le score ZNCC monte quand on dézoome fortement. Toute calibration automatique du zoom doit donc être bornée ou validée autrement que par le score (INFERENCE).
3. **L'appariement échoue vraiment dans une section seulement** : S6 reste à 14–21 % de recalages justes, quel que soit le zoom.
4. **Le recalage à l'estime par caméra est beaucoup plus variable que sur Val.** Sans aucun recalage, l'erreur d'échelle implicite va de −34 % à +19 % et l'erreur de cap de −9,5° à +6,1°, contre −13 % et −2,6° sur Val. Une recherche de 40 m est donc dépassée bien avant 300 m dans plusieurs sections.

**Rejeu complet avec le zoom au meilleur score** (étiqueté « diagnostic, utilise la vérité ») : résultats mitigés. Exemples : S2 en recalage tous les 300 m passe de 120 à 46 m ; S3 en recherche dimensionnée tous les 1000 m passe de 704 à 23 m ; S4 se dégrade de 33 à 301 m. Les sections qui se dégradent sont celles où le zoom tombe au bord 0,40. Cela confirme les points 1 et 2.

## Résultats

### Q2. ALTO
- **MEASURED** : `dl=1` sur le dossier Round 1 renvoie du HTML. L'archive zip de tout le Round 2 (16,6 Go) a calé à 491 Mo et ne peut pas reprendre ; abandonnée.
- **Solution trouvée** : lister le dossier partagé par `POST https://www.dropbox.com/list_shared_link_folder_entries`. Il faut le cookie `__Host-js_csrf` passé dans `t` et dans `X-CSRF-Token`, plus `link_key`, `secure_hash`, `sub_path`, `rlkey` et `link_type=c`. Chaque fichier a alors un lien `…/<fichier>?rlkey=…&dl=1` qui accepte les requêtes Range (réponse 206).
- **Inventaire** : le dossier `6gwa0swtzj7pg1itk89hn` contient les deux tours.

  | Tour | Fichier | Taille |
  |---|---|---|
  | Round 1 (`UAV/`) | readme | – |
  | Round 1 | Train.zip | 10,66 Go |
  | Round 1 | Val.zip | 1,86 Go |
  | Round 1 | Test.zip | 2,04 Go |
  | Round 2 (`UAV_Round2/`) | gt_matches.csv | – |
  | Round 2 | Train.zip | 11,26 Go |
  | Round 2 | Val.zip | 1,86 Go |
  | Round 2 | Test.zip | 3,51 Go |

  Les inventaires des zips sont dans `data/processed/t_inventory/alto_*.csv`.
- **MEASURED, Val** : le Val du Round 2 est identique à celui du Round 1 (mêmes noms de fichiers, mêmes CRC). **`data/raw/alto/Val.zip` est en place** : `unzip -t` ne signale aucune erreur, sha256 `e468050d…`, et le chargeur de l'équipe lit 1684 images.
- **MEASURED, Round 2 Train** : il reprend le Round 1 Train, qui compte 10 436 images sur 28,5 km sans horodatage. Il ajoute environ 9 km vers l'est (E jusqu'à 534 877 m contre 526 084 m) ; 75,8 % de ses points sont à moins de 20 m du Round 1. Pas temporel 0,050 s, aucun trou de plus de 1 s, vitesse médiane 54,6 m/s, altitude 437–548 m au-dessus de l'ellipsoïde. Les références sont décalées de 0, +40 m et −40 m vers le nord, 3744 images chacune.
- `gt_matches.csv` à la racine du Round 2 compte 13 783 lignes : c'est la vérité du Train. **Les deux Test n'ont aucune vérité** et ne servent donc pas à évaluer.
- **PUBLISHED** (https://arxiv.org/abs/2207.12317) : le jeu ALTO complet contient une IMU LCI-1 à 200 Hz, la solution NovAtel SPAN (1,5 m RMS) et un altimètre laser à 20 Hz. Il n'est pas publié : le README de https://github.com/MetaSLAM/ALTO dit « Full Dataset: Coming soon! ». ALTO n'a aucun baromètre.

### Q3. Zurich Urban MAV
- **MEASURED, accès** : le serveur accepte les requêtes Range. `experiments/t_remote_zip.py` lit le répertoire central (ZIP64) : 81 331 fichiers, 29,8 Go. Il extrait ensuite des fichiers choisis en regroupant les voisins dans une seule requête.
  - Débit : 6,6 Mo/s mesurés seuls, environ 1–3 Mo/s pendant les téléchargements Dropbox.
  - Fenêtre extraite : horloge PX4 1795–2405 s, 18 221 images, 6,5 Go, en environ 40 min.
- **Décision : pas de téléchargement complet** (28 Go). La caméra est une GoPro 1920×1080 qui regarde **vers l'avant et sur le côté, à hauteur de rue** (image vérifiée). Le drone est captif et lent (vitesse médiane 0,7 m/s, 1869 m de trajet de vérité en 45 min). Cela ne sert ni la localisation par orthophoto ni le scénario.
- **MEASURED, IMU** : `RawGyro` et `RawAccel` ne sont qu'à 10 Hz et replient les vibrations (le signe de la corrélation s'inverse à ±50 ms). Le gyro à 50 Hz d'`OnboardPose` est le gyro brut tourné, ajusté par moindres carrés : P ≈ A·brut, avec A ≈ [[−0,64, −0,64, 0], [−0,66, 0,67, 0], [0, 0, −0,99]]. Cela correspond à une rotation de 45° avec l'axe z vers le haut, et à une amplitude d'environ 0,65× sur x et y, donc un signal filtré.
- **MEASURED, horodatage des images (nouveau)** : `experiments/t_zurich_sync_check.py` corrèle la vitesse de lacet tirée des images (corrélation de phase) avec le gyro z.
  - Sur toute la fenêtre : meilleur décalage −1,10 s (r = 0,62), contre r = 0,16 à 0 s.
  - Par fenêtres de 20 s : environ +0,2 s jusqu'à 100 s, environ +0,13 s jusqu'à 160 s, puis un saut à −1,24 s qui dérive linéairement jusqu'à −0,90 s à 540 s (+0,95 ms/s, résidu max 0,020 s sur 18 fenêtres), puis un nouveau saut à −1,86 s vers 563 s.
  - INFERENCE : les sauts viennent d'images perdues ou dupliquées dans l'association imgid → horodatage.
  - **Conséquence** : toute fusion caméra/IMU sur AGZ doit corriger ce décalage. La correction du segment stable (183–563 s) est dans `meta.json`.
- La séquence au format commun est `data/processed/t_replay/zurich_agz_1800_2400`. Le validateur répond OK :

  | Fichier | Lignes | Fréquence |
  |---|---|---|
  | IMU | 29 840 | 50 Hz |
  | baro | 5975 | 10 Hz |
  | GNSS dédoublonné | 2988 | 5 Hz |
  | images | 17 921 | 30 Hz |
  | vérité | 597 | 1 Hz |

### Q4. Autres vols réels avec baro brut, caméra et vérité
- **INSANE (AAU Klagenfurt)** : c'est le seul jeu réel trouvé qui réunit une caméra vers le bas, un baro brut et une vérité RTK.
  - Licence « Data: INSANE Dataset; License: BSD-2-Clause », sans droit de vente : https://cns-data.aau.at/insane-dataset/LICENSE.txt.
  - Les capteurs de outdoor_1, mars_1 et mars_2 ont été téléchargés par la piste B. La piste C a ajouté les images de Mars1 (2,4 Go).
  - **Séquence `data/processed/t_replay/insane_mars_1`**, validée (MEASURED) : 100 s, 87 m, 0–5 m de hauteur. IMU 196 Hz, baro 18 Hz, GPS PX4 5 Hz, 1454 images vers le bas à 15 Hz, vérité RTK à 8 Hz, intrinsèques et extrinsèques incluses.
  - **Séquence `data/processed/t_replay/insane_outdoor_1`**, validée (MEASURED, aérodrome de Klagenfurt) : capteurs sur 260 s, images sur 199 s, 3983 images vers le bas à 20 Hz (sol texturé et ombre du drone), 187 m, 0–24 m au-dessus du départ.
  - Limite d'outdoor_1 : **RTK fixe 21,9 % du temps seulement**, avec un trou de vérité fixe allant jusqu'à 98,6 s.
  - Décalage images/gyro global d'outdoor_1 : −0,04 s (axe gy, r = −0,69). Par fenêtre, l'estimation est instable, car la caméra vers le bas donne un signal de lacet faible.
  - Inventaire des 20 séquences (0,01–12,6 Go) dans `data/processed/t_insane/listing.csv`.
- **PX4 Flight Review** (journaux « CC-BY PX4 », https://review.px4.io/browse) : 471 956 journaux listés, 26 téléchargés (2,9 Go), 25 avec `sensor_baro`. Un seul déclenchement caméra et aucune capture ; aucune image dans les journaux.
  - Exports `data/processed/t_px4/<id>/` avec un schéma voisin du format commun.
  - Journaux RTK fixe : `53736001…` (94 %, 2334 s, mais IMU journalisée à environ 4 Hz) et `036fb3a7…` (84 %, 1393 s, IMU 200 Hz). Transmis à la piste B.
- **Vidéos liées aux journaux PX4 (nouveau)** : le champ `video_url` de `dbinfo.json` contient 114 URL vidéo distinctes, triées dans `data/processed/t_px4_video/candidates.csv`.
  - Deux vols EasyStar (voilure fixe, caméra FPV avant) sont devenus des séquences : `px4video_d4cc6eb1` (820 s, montée de 296 m au baro) et `px4video_aa0ae4df` (1262 s).
  - **MEASURED, contre-vérification indépendante sur `d4cc6eb1`** : décalage 0,02 s, r = −0,79. Par fenêtres de 100 s, de −0,10 à +0,10 s, avec |r| entre 0,75 et 0,89. Commande : `.venv/bin/python experiments/t_zurich_sync_check.py data/processed/t_replay/px4video_d4cc6eb1 100`.
  - `aa0ae4df` : r = 0,43 avec un second pic à 0,33, donc ambigu.
  - Limites : baro journalisé à 1 Hz seulement ; licence des vidéos inconnue.
- **Autres jeux (PUBLISHED, recherche par sous-agent, citations dans le manifeste)** : MARS-LVIG, MUN-FRL, VPAIR, UAV-VisLoc, AerialVL, AnyVisLoc, UAVD4L et AerialExtreMatch ont une caméra nadir mais aucun baromètre listé. NTU VIRAL, Blackbird, FusionPortable et GND ne conviennent pas.
  - Les jeux de photos DJI `tuniu_tw_1/2` (Taïwan, RTK) existent dans l'index ODM. Leur `RelativeAltitude` est une altitude **fusionnée** d'après la documentation DJI (https://developer.dji.com/onboard-sdk/documentation/guides/component-guide-altitude.html). Ils serviraient donc à la piste A, pas pour le baro.
  - Requêtes en chinois simplifié et traditionnel : aucun jeu taïwanais avec baro brut trouvé dans ce budget de recherche (8 requêtes). Ce n'est pas une preuve d'absence.

### Q5. Simulation : correctif proposé (pas appliqué à la branche de l'équipe)
Le diff `data/processed/t_sim_rec/sim_patch.diff` (706 lignes) porte sur une copie de travail, `data/raw/t_sim_work/sim`. L'instantané de l'équipe n'a pas été modifié. Contenu :
1. **Fuite d'orientation corrigée** : `sensor_noise.py` remet le quaternion à zéro, en plus de la covariance −1. Vérifié avec `ros2 topic echo` : orientation 0/0/0/0.
2. **Coupure GNSS** : nouveau nœud `gnss_gate.py`. Le bridge envoie `/sim/gps_raw`, qui est republié sur `/gps/fix` tant que le temps simulé est inférieur à `gnss_cut_s`.
3. **Stéréo optionnelle** : `stereo:=true` ajoute une 2ᵉ caméra bas décalée de 0,30 m, via une variante temporaire du SDF.
   - À 60 m, la disparité attendue est de 256 × 0,30 / 60 = **1,28 px**, sous le bruit d'appariement (INFERENCE). Une stéréo sur le drone n'apporte pas l'échelle à cette altitude ; seule une base multi-vues le peut (cohérent avec Song et al. 2017).
4. **Enregistreur** `recorder.py`, qui écrit directement au format commun.
5. **Scénario** `t_scenario.py` : montée à 60 m, 120 s à 8 m/s, virage de 90°, 60 s, avec correction d'altitude et de cap. Le premier essai en boucle ouverte avait touché le sol à 156 s.

Résultat SIMULATED : `data/processed/t_sim_rec/t_sim_terrain_cut60`, 255 s, 1,36 km, 467 Mo, validé.
- Dernière mesure GNSS à t_s 53,27.
- Facteur temps réel 0,57–0,95 avec 2 caméras de 512 px.
- Écart baro − vérité : écart-type 0,98 m, 1,45 m en fin de vol (bruit de 10 Pa et dérive du modèle de l'équipe).
- **Défaut corrigé à la main** : `meta.gnss_cut_s` valait 60 (temps simulé) alors que l'origine t_s est à 5,73 s. Je l'ai corrigé à 54,27. Le correctif doit écrire la coupure dans le repère t_s.

**Comparaison avec le rejeu Python** (`experiments/t_gen_wufeng_replay.py`) : Python produit l'imagerie réelle de Taïwan, en déterministe, en quelques minutes, sans Docker. Ses limites : sol plat, pas de parallaxe, pas de dynamique de vol. Gazebo apporte la dynamique, le relief 3D (arbres) et la boucle fermée, mais son terrain est procédural ou plaqué et son facteur temps réel est inférieur à 1. Aucun autre simulateur n'apparaît clairement utile cette nuit.

### Q6. Format commun et chargeurs
- Format `taipeidrift-replay/1`, défini dans `experiments/t_replay.py` :
  - Fichiers : `meta.json`, `imu.csv`, `baro.csv`, `gnss.csv`, `images.csv`, `truth.csv` (évaluation seulement).
  - Une seule horloge ; la provenance de chaque capteur et le sens de l'altitude baro sont obligatoires.
  - `load(seq, cut_s)` retire le GNSS après la coupure ; `load_truth` est séparé.
  - Contrôle : `validate`.
- Exporteurs et générateurs : `t_export_zurich.py`, `t_export_insane.py`, `t_px4_export.py`, `t_gen_wufeng_replay.py`, plus le `recorder.py` de Gazebo.
- Neuf séquences valident OK : Zurich, INSANE Mars1 et outdoor_1, 2 vidéos PX4, 2 Wufeng, Gazebo.
- **SIMULATED, contrôles Wufeng** : sans bruit, l'IMU intégrée en navigation inertielle donne 0,05 m d'erreur à 60 s, ce qui confirme des repères cohérents. Bruit blanc baro 0,67 m. Pixels noirs dans les images : 0,001 %.

## Commandes de vérification
```
.venv/bin/python experiments/t_replay.py validate data/processed/t_replay/* data/processed/t_sim_rec/t_sim_terrain_cut60
.venv/bin/python experiments/t_remote_zip.py list https://download.ifi.uzh.ch/rpg/AGZ_data/AGZ.zip
.venv/bin/python experiments/t_zurich_sync_check.py data/processed/t_replay/zurich_agz_1800_2400 20
.venv/bin/python experiments/t_export_zurich.py --t0 1800 --t1 2400 --images "data/raw/zurich_mav/AGZ_window/AGZ/MAV Images" --out data/processed/t_replay/zurich_agz_1800_2400
```

## Questions ouvertes (aussi dans `questions.md`)
- Licence des vidéos YouTube liées aux journaux PX4.
- Usage des images AGZ dans la vidéo du jury.
- Application du correctif du simulateur par son auteur.

## Prochaines expériences
1. Piste A : brancher d'autres appariements (XFeat + RANSAC) dans `experiments/t_alto_heldout.py`, en gardant le même protocole gelé et les mêmes 8 sections du Round 2.
2. INSANE outdoor_1 : baro brut contre RTK, mais seulement sur les périodes en RTK fixe. Tester aussi le flux optique de la caméra bas au-dessus de l'herbe et de la piste.
3. Corriger le recorder (coupure en t_s) et ajouter le relief Copernicus au monde Gazebo.
4. Télécharger un jeu ODM `tuniu_tw` pour tester la piste A sur de vraies photos de drone à Taïwan contre NLSC/OAM.
5. Rendre la calibration du zoom robuste : plus de recalages pendant la phase GNSS, zoom borné, et validation par la cohérence entre recalages successifs plutôt que par le score.
   - Attention : le diagnostic a déjà regardé les 8 sections. Il n'y a donc plus de données ALTO jamais vues avec vérité ; le Round 1 Train est inclus dans le Round 2 et les Test n'ont pas de vérité.
   - Un test propre demande un autre vol nadir avec vérité : MUN-FRL (CC BY 4.0, RTK/PPK) ou MARS-LVIG (CC BY-NC-SA, RTK).
