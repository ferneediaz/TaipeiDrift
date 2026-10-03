# Avis externes sur la stratégie (3 octobre 2026)

Deux modèles ont reçu le même contexte : résultats de la nuit, critères du jury, données disponibles. Ils ont écrit leur propre plan avant de lire le nôtre, puis l'ont critiqué. Textes bruts, non édités.

- Grok 4.7 (xAI), raisonnement maximal
- GPT-6 Astra (OpenAI), raisonnement maximal

## Grok 4.7

## 1. Ce que je construirais

Pas une navigation continue. Le produit montré : un reset de position, accepté seulement s’il ne ment pas ; sinon un budget de dérive. Trois couches, jamais fondues en un faux vol.

**(a) Données.**

1. MESURÉ — baisse d’erreur. ALTO Round 2 Train, 8 sections, 37,4 km, paramètres gelés (PUBLIÉ : hélico, Ohio, https://arxiv.org/abs/2207.12317). Caméra et carte réelles. Vérité réservée aux graphes. Baseline sur la même trajectoire : dead-reckoning caméra, 219 m de médiane par section de 4,6 km. ZNCC tous les 300 m : 94 m (20–339). Ce n’est pas « zéro faux » : le zoom calé sur trois recalages d’avant-coupure est souvent mauvais ; un meilleur zoom fait passer les bons recalages de 21–43 % à 86–100 % sur 3 sections. Le 485 m / 78 s (Mid-Air, jeu synthétique) n’est pas la baseline : autre engin.

2. MESURÉ, à part. Baro Zurich, INSANE, PX4 RTK : 0,6 m à 60 s, 1,8 m à 5 min, 4,5 m à 20 min ; 0,30 m + 0,112 m/√s. Échelle fausse de 3–7 % sans calage sous GNSS. Ces résidus justifient un bruit. Ils ne se collent pas aux photos ALTO dans le même chiffre.

3. SIMULÉ, second chiffre. Images ALTO réelles. Baro = altitude vraie + résidus Zurich rejoués (≥20 tirages, graine gelée). DEM lu à la position estimée, pas vraie. L’estimateur ne voit pas l’altitude vraie. Pas d’IMU obtenue en dérivant la vérité : la forme du signal est la vérité, un filtre la retrouve. Zoom oracle et vitesse caméra oracle (608 → 74 m) : hors vidéo, étiquette ORACLE.

INSANE (0–24 m, sans orthophoto) prouve le baro, pas la nav. Pas de téléphone, pas de MARS-LVIG dans le délai. Chaque graphe : MESURÉ, SIMULÉ ou ORACLE, le jeu, l’altitude, le taux d’acceptation à côté de la médiane.

**(b) Architecture, pour ce jury.**

GNSS seulement avant la coupure : position, échelle baro, zoom, puis gel. Dead-reckoning caméra. Zoom du template = baro − DEM. ZNCC, recherche lacet et échelle. Acceptation : quad ≥ 3. Gate de dérive seulement sur les runs déjà faits, étiqueté SIMULÉ. Sortie FIX / COAST / PERDU.

Je garde baseline, ZNCC, quad ≥ 3, modèle baro, couverture calculée 21 % / 72 % / 9 % (Copernicus + WorldCover, pas un vol), liste des idées tuées. Je ne build pas XFeat (0/300), le soleil (−6 % d’erreur finale), la stéréo, OSM, les ombres, le magnétisme, ni Gazebo comme preuve (fuite d’attitude déjà vue).

Barème : baisse d’erreur, validité, bruit, calcul, déploiement. Un Kalman joli ne départage pas. Un taux de faux nul sur du nadir, un échec de zoom expliqué, du CPU sans GPU, et une carte qui dit non sur 9 % de Taïwan (37 % sur l’agglo de Kaohsiung) : un juge défense peut s’en servir. Utilisateur : mission d’un multirotor ~500 USD, GNSS au décollage, brouillé ensuite, qui doit refuser une position. ALTO (~450 m, 55 m/s) est le cas dur, pas l’engin. INFÉRENCE sur le profil jury ; les erreurs citées sont MESURÉES.

**(c) Trois démos.**

1. Les 8 sections : trajectoire et erreur dans le temps, DR contre recalages, section à 339 m comprise. Titre : 219 → 94 m de médiane par section, paramètres gelés, part de recalages faux écrite.
2. Une image acceptée (4 sous-gabarits au même point) contre une rejetée. 44/300, médiane 10 m, max 18 m, 0 faux. Recadrages synthétiques d’orthophotos réelles : 0 faux sur 2 395 groupes négatifs. Sans gate, SIMULÉ : 1 043 des 2 557 recalages acceptés dépassent 25 m.
3. Enveloppe : carte 21/72/9, erreur depuis le dernier recalage (240 s, SIMULÉ : 18,9 m contre 220,7 m), courbe baro réelle.

**(d) Cinq façons de passer pour du vent.**

1. Ouvrir sur 26–31 m. C’est la section de réglage. Ouvrir sur 94 m et la fourchette.
2. Écrire « ALTO completed » avec une IMU dérivée de la vérité. Elle ne touche pas le chiffre du titre.
3. Un seul chiffre qui mélange photos réelles et baro simulé. Deux chiffres, deux étiquettes.
4. Cacher 44/300. La thèse : on préfère se taire qu’inventer une position.
5. « Zéro faux » sans domaine. OrthoLoC, oblique ~100 m : 3 négatifs acceptés sur 3 540, et 3 à 5 acceptations sur 60. Dire l’écart hélico / drone avant la question. Vidéo : une section entière, rejet inclus.

## 2. Critique du plan

Juste : baro − DEM → zoom → ZNCC → quad ≥ 3 → budget de dérive colle à la cause mesurée. Vérité au score seul, DEM à la position estimée, résidus Zurich rejoués plutôt qu’un gaussien, ablations et balayages de bruit : c’est le barème. Stockage carte et compagnon PX4 : une slide, pas du code.

Trop gros, et dangereux. « ALTO completed » est le piège. Dériver des poses lissées et coller une variance d’Allan de MUN-FRL ou d’INSANE ne fait pas un hélico à 55 m/s. Le basse fréquence reste la vérité. Si cette attitude redresse l’image, c’est un oracle : OrthoLoC montre qu’1° change le résultat. Je coupe l’IMU synthétique. Le terme de vitesse sur le baro aussi : mesuré jusqu’à 10 m/s, confondu avec l’altitude.

Le soleil sort du build. −6 %, soleil sous 70°, ciel clair ~40 % du jour à Taichung en octobre. Une ligne dans les pistes tuées.

Recherche globale et fenêtre qui grossit : 40,6 % et 0/143 faux, SIMULÉ, 23 km². Si le script tourne, une courbe. Sinon on ne l’écrit pas. Pas de nouveau Kalman en 36 h.

Téléphone à Taïwan, OrthoLoC relancé, MARS-LVIG : deuxième thèse. Le téléphone n’est pas nadir ; droits NLSC non confirmés ; MARS-LVIG, 8–30 Go la séquence, sans baro. OrthoLoC est déjà le hors-spec. On le cite.

« Petite carte » : aucune dans le brief. Chronométrer le Mac, nommer la puce.

Manques. Le plan fond deux protocoles. 94 m : recalage forcé tous les 300 m, des faux dedans, hors réglage. Quad ≥ 3 : 0 faux, peu d’acceptations ; l’essai baro+quad sur ces sections a déjà été inspecté (70 m contre 128 m, 10/118, pas held-out). Le jury n’en retiendra qu’un. Les séparer. Manque le calage d’échelle baro sous GNSS, l’écart de domaine, l’âge de carte Wufeng 2018/2020 (meilleure preuve Taïwan déjà sur disque ; images réelles, capteurs à étiqueter SIMULÉ), et l’interdit d’afficher 26 m.

Couper jusqu’à : rejeu ALTO gelé, rejeu zoom baro sans IMU neuve, trois figures, une slide.

## 3. 36 heures, six personnes

Heures 0–2, tout le monde : geler le pitch, la légende, le storyboard. Interdit dans le titre : 26 m, « ALTO completed », soleil, XFeat, Gazebo, téléphone.

En parallèle, environ 8 h :
1. Rejeu des 8 sections, figures, aucun re-réglage.
2. Zoom baro, ≥20 résidus, DEM à la position estimée. Pas fini à H+10 : la vidéo part sans ce chiffre.
3. Table d’intégrité et une paire d’images. Aucun essai nouveau après H+12.
4. Carte 21/72/9, Mo/km² sur les tuiles disque. Wufeng seulement si le chiffre existe déjà.
5. Script calé sur les fichiers présents.
6. Chrono ZNCC, slide PX4, veto sur tout nombre sans étiquette.

H+14 : bout-à-bout, on cherche la fuite. Jour 2, environ 6 h : voix, export, une page (instant de coupure, commit des paramètres, ce qui est simulé). Arrêt des essais 4 h avant le rendu. La personne 6 ne code pas de feature.

Pitch : « Après la coupure GNSS, on ne publie une position corrigée que si le recalage sur l’orthophoto embarquée est d’accord avec lui-même — environ 10 m et zéro faux parmi 300 images nadir réelles, 44 acceptées — sinon on annonce la dérive, 219 m de médiane par section de 4,6 km sur 37 km hors réglage. »

Prochaine action, 10 min : écrire cette phrase, barrer « 26 m », lancer le rejeu des 8 sections sans toucher un paramètre.

[You have received this identical output 4 times. Re-reading 'agent://AdvisorGrok/answer' will not change it — use a narrower selector (path:A-B), or proceed with the edit.]

## GPT-6 Astra

## 1. Construisez un rejeu causal, pas une promesse de drone autonome

**[INFERENCE] Proposition formulée avant lecture de votre plan, condensée ici :** une seule chaîne, deux preuves séparées : navigation visuelle sur vol réel ; intégration multisensorielle simulée.

### a. Assembler les données honnêtement

**[PUBLISHED]** Sources : [ALTO](https://github.com/MetaSLAM/ALTO), [INSANE](https://arxiv.org/html/2210.09114), [OrthoLoC](https://deepscenario.github.io/OrthoLoC/). **[MEASURED, équipe]** Aucun jeu disponible ne couvre votre chaîne complète à l’altitude cible.

**[INFERENCE]** ALTO prouve caméra→estime→carte ; INSANE/PX4 caractérisent les capteurs ; OrthoLoC expose le changement de domaine. Ne fusionnez pas leurs flux comme un même vol. Pour l’intégration complète, réutilisez Wufeng : caméra synthétisée depuis l’orthophoto 2020, carte 2018, trajectoire et capteurs simulés. Étiquetez « SIMULATED », sol plat et absence de parallaxe ; ce n’est pas un vol taïwanais.

Baromètre : bruit blanc **0,30 m**, marche aléatoire **0,112 m/√s**, rampe par vol **0,0024 m/s**, paramètres issus des journaux [MEASURED]. Préservez la corrélation temporelle, comparez les distributions à 60/300/600 s ; utilisez 20 graines et un scénario bruit doublé [INFERENCE]. Pour l’IMU, caractérisez biais/bruit sur les séquences statiques INSANE ; générez accélération spécifique et rotations avec gravité et repères corrects. Tout paramètre non caractérisé reste une hypothèse.

**[INFERENCE]** Séparez générateur, estimateur et évaluateur. L’estimateur ne reçoit jamais la vérité. Calibration uniquement avant coupure GNSS ; ensuite, recherche carte, lecture DEM et cadence des corrections dépendent de l’estimation, pas de la référence. Vérifiez horloges, extrinsèques et référentiels altimétriques. Test anti-fuite : une fois les capteurs générés, retirer la vérité ne change aucune estimation.

### b. Architecture et intérêt pour ce jury

**[INFERENCE]** Gardez la propagation existante : déplacement caméra, gyro pour l’attitude, baro−DEM pour contraindre l’échelle ; ZNCC avec recherche lacet/échelle, « quad ≥ 3 » et cohérence temporelle pour accepter les corrections. Aucun nouveau SLAM. Affichez position, âge du dernier recalage et état « corrigé / à l’estime / indisponible » ; pas de confiance probabiliste inventée.

**[MEASURED]** ALTO gelé : médiane des médianes de section **219→94 m**, corrigée **20–339 m** selon section. ZNCC accepte **44/300** images réelles ; XFeat **0/300**. Gardez ZNCC ; coupez XFeat, solaire, stéréo et Gazebo du chemin critique. Le gain de calibration utilisant toute la vérité n’est pas déployable.

**[INFERENCE]** Même estime avec/sans correction pour noter l’erreur ; dégradations pour la robustesse ; latence p95, RAM et stockage pour l’intégration. Utilisateur : opérateur de drone civil d’inspection après perte GNSS. Déploiement : caméra calibrée, horloges synchronisées, tuiles locales licenciées. Un Mac ne prouve ni l’embarqué ni un système complet à 500 $.

### c. Les trois vues décisives [INFERENCE]

1. **ALTO :** trajectoires et erreur temporelle, coupure et corrections visibles ; résultats des huit sections, pas seulement la meilleure.
2. **Wufeng :** coupure GNSS puis masquage caméra 60 s ; reprise, médiane/p95 sur 20 graines, bruit doublé. Bandeau « SIMULATED » permanent.
3. **Intégrité :** vrai recalage accepté et faux candidat rejeté ; disponibilité et faux acceptés sur ALTO/OrthoLoC.

### d. Prévenir le « bullshit »

**[MEASURED]** Les 26–31 m sont réglés sur Val ; les 70 m ultérieurs sont exploratoires ; OrthoLoC comporte **3 négatifs acceptés/3 540 paires**. Ni « généralise à 30 m », ni « zéro faux garanti ». « Quad » conserve des seuils géométriques.

**[INFERENCE]** Baro−DEM n’est pas une localisation terrain indépendante. Forêt, mer et nuit restent des limites ; couverture cartographique ne signifie pas réussite en vol.

[You have received this identical output 3 times. Re-reading 'agent://AdvisorAstra/partie_1' will not change it — use a narrower selector (path:A-B), or proceed with the edit.]

## 2. Gardez le cœur du plan ; retirez « ALTO complété » comme preuve réelle

**Plan maintenant lu. Avis [INFERENCE] :** l’architecture est cohérente ; le programme expérimental dépasse ce qui est nécessaire pour convaincre.

1. **Juste :** calibration avant coupure, redressement, échelle barométrique, corrections contrôlées, ablations et budget de dérive. Le contrôle temporel complète utilement « quad », mais peut aussi verrouiller une mauvaise estimation : montrer une perte de recalage, pas seulement une récupération réussie.
2. **Trop ambitieux :** synthétiser l’IMU ALTO puis estimer sa stabilité de biais par Allan. Des poses lissées différenciées ne reproduisent ni vibrations ni dynamique rapide ; ajouter du bruit ne rétablit pas ces informations. Une Allan exploitable exige des séquences statiques suffisamment longues. Gardez ALTO avec baro synthétique déjà disponible ; démontrez l’intégration IMU sur Wufeng. « Vérité seulement pour scorer » devient « vérité réservée au générateur et à l’évaluateur ».
3. **Manquant :** conversion ellipsoïde/géoïde/altitude barométrique, erreur initiale GNSS, incertitude DEM et corrélation entre hauteur, position et échelle. Baro−DEM à une position erronée peut renforcer l’erreur. Les résidus Zurich ne valident pas la pression dynamique d’ALTO à 55 m/s. Publiez aussi p95, pire section, faux recalages et durée sans correction, avec comparateurs identiques.
4. **À couper :** solaire, recherche globale, balayages combinatoires, nouveau MARS-LVIG. OrthoLoC déjà examiné n’est plus un test vierge ; un téléphone au sol n’établit pas la navigation aérienne. Pour le calcul embarqué, mesurez sur carte seulement si elle est disponible ; sinon dites « Mac mesuré, embarqué non validé ». Réservez le temps à la vidéo et aux droits des cartes.

## 3. Affectez maintenant six responsables ; verrouillez la vidéo avant H36

**Organisation proposée [INFERENCE] — H0 = maintenant.**

| Responsables | Livrable personnel |
|---|---|
| P1 / P2 | P1 : rejeu, provenance, séparation vérité ; P2 : redressement, ZNCC et refus des faux candidats. |
| P3 / P4 | P3 : calibration pré-coupure, propagation et baro−DEM ; P4 : comparaisons, 20 graines et scénarios dégradés. |
| P5 / P6 | P5 : latence, RAM, stockage, licences et contre-audit ; P6 : scénario utilisateur, captures, montage et export. |

1. **H0–H2 :** tous figent interfaces, paramètres, trois vues et critères de réussite ; P6 commence le storyboard.
2. **H2–H12 :** P1–P3 raccordent la chaîne ; P4 prépare les évaluations ; P5 contrôle les entrées ; P6 assemble une première vidéo. À H12 : gel fonctionnel.
3. **H12–H20 :** calculs lancés, huit heures de repos ; aucune nouvelle fonctionnalité.
4. **H20–H28 :** P4/P5 évaluent et contre-auditent ; P1–P3 corrigent uniquement les défauts ; P6 monte les résultats réels.
5. **H28–H36 :** capture définitive, questions hostiles, export ; dernières deux heures réservées à la lecture intégrale et au dépôt.

**Pitch :** « TaipeiDrift limite la dérive après perte GNSS par recalage visuel sur cartes hors ligne, et rend visible quand ses observations ne suffisent plus. »
