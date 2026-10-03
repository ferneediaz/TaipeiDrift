# Questions pour demain

Questions que seule l’équipe peut trancher. Chaque fois, la nuit a continué avec l’hypothèse indiquée.

## Orchestration

1. ~~**ALTO Val.zip**~~ — résolu dans la nuit : la piste C l’a récupéré par le nouveau lien Dropbox (`data/raw/alto/Val.zip`, 1,86 Go), ainsi que `UAV_Round2_Train.zip` (11,3 Go). Rien à faire, sauf si Dustin veut comparer les sommes de contrôle avec sa copie.
2. **Matériel visé** : pour le budget d’environ 500 dollars, avez-vous déjà un modèle de drone ou de caméra stéréo ? La bande d’altitude couverte par la stéréo dépend directement de l’écart entre les deux caméras.
   Hypothèse retenue : on compare plusieurs caméras stéréo du commerce, sans en choisir une.
3. **Altitude de vol de la démo** : vise-t-on plutôt 30 à 50 m, ou 100 à 150 m au-dessus du sol ?
   Hypothèse retenue : les deux bandes sont testées séparément.
4. **Droits de l’imagerie NLSC** : quelqu’un peut-il confirmer avec les organisateurs ou le NLSC que les tuiles peuvent être stockées hors ligne pour la démonstration ?
   Hypothèse retenue : seuls quelques échantillons sont utilisés en recherche ; les droits sont documentés et non supposés.
5. **Jeu de données des organisateurs** (IMU, vitesse, cap, position de référence d’une plateforme en mouvement) : le lien est sur la page réservée aux participants. Il n’est pas public et n’a pas été récupéré. Qui peut le déposer dans `data/raw/organiser/` ?
   Hypothèse retenue : rien n’a été fait dessus ; c’est probablement le jeu sur lequel le jury attend la ligne de base.

## Piste B : hauteur, vitesse et cap

1. **Date et heure de la démo ou du vol filmé** : le capteur solaire ne sert à rien soleil presque au zénith (mai à août vers midi à Taïwan) et peu sous ciel couvert. Quel mois et quelle heure ?
   Hypothèse retenue : octobre, entre 9 h et 16 h, ciel variable.
2. **Centrale d'attitude** : le drone a-t-il un magnétomètre utilisable et quelle erreur de roulis/tangage annonce l'autopilote en vol ? L'erreur de cap du capteur solaire vaut environ tan(élévation) x erreur d'inclinaison.
   Hypothèse retenue : autopilote PX4/ArduPilot standard, 1 à 2 degrés d'erreur d'inclinaison en vol.
3. **Relief sous la trajectoire** : vol de démo en plaine (Taichung, autoroute de Wufeng) ou en relief ? Le baromètre ne suit la hauteur sol que si le terrain est plat ou si un modèle d'élévation embarqué est autorisé.
   Hypothèse retenue : plaine, modèle d'élévation Copernicus GLO-30 embarqué autorisé.

## Piste C : données, rejeu et simulation
- Vidéos YouTube liées aux journaux PX4 (EasyStar FPV, `data/raw/px4_video/`) : licence non établie. Pouvons-nous montrer ces images dans la vidéo du jury, ou seulement les utiliser en interne ? Hypothèse retenue : usage interne seulement.
- Zurich AGZ : images « academic research without any limitations ». Une vidéo de jury en hackathon compte-t-elle comme recherche académique ? Hypothèse retenue : oui avec citation, à confirmer.
- Le simulateur : voulez-vous que le correctif `data/processed/t_sim_rec/sim_patch.diff` (coupure GNSS, enregistreur, stéréo, fuite d'orientation) soit appliqué sur la branche `simulations` par son auteur ? Rien n'a été modifié sur la branche.
