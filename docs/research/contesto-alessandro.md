# Contesto per Alessandro: cosa c'è in questo branch e come usarlo per il VIO

Branch `research/offline-nav-evidence`, scritto la notte tra il 2 e il 3 ottobre 2026 a partire da `main` 1286d5c. Questo file è in italiano; il resto della ricerca notturna in `docs/research/` è in francese. Le sezioni che ti servono sono riassunte qui, con i numeri.

Etichette usate ovunque:
- **MISURATO**: girato su dati reali;
- **SIMULATO**: generatore con ipotesi dichiarate;
- **PUBBLICATO**: affermazione di un articolo, con link;
- **INFERENZA**: nostro ragionamento non verificato.

## 0. Da dove partire (10 minuti)

1. `git fetch && git switch research/offline-nav-evidence`
2. `uv sync --extra research`. Il gruppo opzionale `research` aggiunge torch CPU, pyulog, rosbags, pyshp e tqdm; non serve per il barometro.
3. Leggi le sezioni 3 e 4 qui sotto: modello del barometro reale e dati reali per il VIO.
4. Proposta di lavoro: sezione 7.

I dati scaricati e i risultati (`data/raw/`, `data/processed/`) **non sono nel repository**: sono su un portatile, a parte. La sezione 4 dice come riscaricarli o rigenerarli. Per avere subito i file pronti, chiedi a Ilhan una copia di `data/processed/t_replay/insane_mars_1` (2 MB, ma le immagini a cui rimanda pesano 4,5 GB).

## 1. Il progetto in breve

- **Challenge 2**: navigare dopo la perdita del GNSS. Il GNSS c'è al decollo, poi sparisce; si stima la posizione dopo. Solo software, per un drone low-cost (circa 500 USD) con camera verso il basso, IMU e barometro. Niente LiDAR. Mappe e modelli a bordo, offline.
- **La giuria vuole**:
  - una baseline di navigazione stimata (dead reckoning) ;
  - almeno una correzione o una fusione di sensori ;
  - grafici della traiettoria stimata contro quella di riferimento, e dell'errore nel tempo ;
  - i limiti quando i sensori cedono o il rumore cresce.
- **Criteri di punteggio**: riduzione dell'errore di posizione, validità tecnica, tolleranza al rumore, requisiti di calcolo e integrazione, fattibilità del dispiegamento. Contano anche l'utente, il dispiegamento e la scalabilità.
- **Video finale preregistrato.** Si possono rigiocare voli registrati o simulati, purché etichettati.

## 2. Dove sono gli altri (fetch del 3 ottobre, ore 10)

| Chi | Branch | Stato |
|---|---|---|
| Tu | `mid-air-vio` | ESKF VIO su Mid-Air : 13–33 m dopo 83 s senza GNSS su 3 voli non usati per la taratura, contro 116–441 m con IMU + barometro e 338–912 m con IMU sola. Barometro simulato. |
| Dustin | `alto-navigator` | Navigatore camera su ALTO come codice condiviso (`baseline/`, 91 test), con incertezza dichiarata e stato tracking / degraded / lost. Ha trovato un errore nel protocollo ALTO (sezione 6). Accordo tra 3 frame vicini (14 m) provato e scartato: frame vicini sbagliano nello stesso posto. Il suo test su dati mai visti è bloccato perché gli mancano le posizioni del Train ALTO; questo branch le ha (sezione 5). |
| Felix | `main`, cartella `TRN/` | Piano di un simulatore di navigazione sul rilievo con telemetro laser, sui modelli di elevazione a 20 m del Ministero dell'Interno taiwanese. Aspetta approvazione prima di scrivere codice. |
| Questo branch | `research/offline-nav-evidence` | Ricerca notturna: dati, esperimenti, idee scartate, proposta per la giuria. |

Decisione aperta del team: quale baseline IMU va su `main`, `mid-air-baseline-fix` oppure il tuo `mid-air-vio`. Hanno la stessa regola del giroscopio.

## 3. Il barometro reale: il modello da mettere nel tuo ESKF

### 3.1 Da dove viene (MISURATO)

| Fonte | Sensore | Riferimento | Script |
|---|---|---|---|
| Zurich Urban MAV, 45 min, volo legato (Fotokite) | Barometro Pixhawk, 10 Hz | Posizioni della camera da fotogrammetria Pix4D, 1 Hz | `experiments/p_zurich_baro.py`, `experiments/s_zurich_vertical.py` |
| INSANE | MS5611 nel PX4, circa 18–20 Hz | RTK fisso | `experiments/s_insane_baro.py` |
| 2 log pubblici PX4 con RTK | Barometro PX4 | RTK fisso | `experiments/s_px4_rtk_baro.py` |

La colonna « altitude » del barometro di Zurich è esattamente la formula dell'atmosfera standard applicata alla pressione, a 3 mm. È quindi un barometro grezzo, non un'altitudine fusa.

### 3.2 I numeri

Modello adattato su Zurich (`data/processed/zurich_vertical/baro_error_model.csv`, `experiments/s_zurich_vertical.py`), confermato da INSANE tra 10 e 120 s:

- rumore bianco **0,30 m** ;
- random walk **0,112 m/√s** ;
- deriva lineare per volo **0,0024 m/s**.

Errore sull'altitudine relativa dopo un taglio del GNSS, Zurich, finestre sovrapposte di un solo volo:

| Dopo il taglio | Mediana | p95 |
|---|---|---|
| 10 s | 0,41 m | 1,42 m |
| 60 s | 0,58 m | 2,12 m |
| 2 min | 0,82 m | 2,97 m |
| 5 min | 1,75 m | 4,04 m |
| 10 min | 2,49 m | 6,11 m |
| 20 min | 4,50 m | 8,83 m |

Per confronto, l'altitudine GNSS dello stesso volo ha una mediana di 1,8–4,6 m e un p95 di 6,9–22 m.

Note:
- i due log PX4 con RTK derivano circa **2 volte meno** a 600 s (mediana circa 1 m contro 2,45 m) : il modello è pessimista, quindi prudente ;
- errore di scala del barometro misurato tra il 3 e il 7 %, nei due sensi ;
- effetto della velocità : circa 0,7 m a 10 m/s, ma su **un solo log**. Non è validato oltre.

### 3.3 Come usarlo

Esempio di generatore, solo dal lato simulazione. L'ESKF vede solo `baro`, mai `alt_true` :

```python
import numpy as np

def simulated_baro(t_s, alt_true_m, rng, white=0.30, rw=0.112, ramp_sigma=0.0024):
    """Barometro SIMULATO calibrato su log reali (Zurich, INSANE, PX4 RTK)."""
    dt = np.diff(t_s, prepend=t_s[0])
    walk = np.cumsum(rng.normal(0.0, rw * np.sqrt(np.maximum(dt, 0.0))))
    ramp = rng.normal(0.0, ramp_sigma) * (t_s - t_s[0])
    return alt_true_m + rng.normal(0.0, white, size=t_s.shape) + walk + ramp
```

Protocollo che reggerà davanti a giurati esperti :
1. almeno **20 seed** per volo ; riporta mediana e p95, non un solo run ;
2. tre livelli di rumore, **×0,5, ×1 e ×2**, per la tolleranza al rumore, che è un criterio di punteggio ;
3. etichetta ogni grafico « barometro SIMULATO, calibrato su log reali ».

### 3.4 Due avvertenze

- **L'IMU non riduce la deriva in quota.** Su INSANE, con una vera IMU a 196 Hz, la fusione IMU + barometro liscia il rumore di circa 1,6 volte, ma la deriva a 60 s resta la stessa (`experiments/s_insane_vertical.py`, MISURATO). Su Zurich l'IMU non aggiunge niente: accelerometro a 10 Hz senza preintegrazione.
- **Mai sostituire il barometro con un'altitudine ricavata dal video.** In simulazione sottostima la deriva di **2,7–3 volte** (`docs/research/sensor-fusion.md`).

## 4. Dati reali per provare il VIO fuori da Mid-Air

Mid-Air è sintetico e i giurati lo noteranno. Nessun dataset pubblico trovato ha tutto insieme (camera verso il basso, barometro grezzo, IMU, verità e ortofoto a 100 m o più) ; questi sono i più vicini al tuo ESKF.

### 4.1 Formato comune `taipeidrift-replay/1`

Definito e validato da `experiments/t_replay.py`. Una cartella per sequenza :

```
meta.json    provenienza per sensore, assi, origine, licenza, etichetta MEASURED/SIMULATED
imu.csv      t_s, gx, gy, gz [rad/s], ax, ay, az [m/s^2]   (assi indicati in meta.sensors.imu.frame)
baro.csv     t_s, pressure_pa, temperature_c, alt_isa_m   (alt_isa_m = ISA con p0 = 101325 Pa)
gnss.csv     t_s, lat_deg, lon_deg, alt_m, fix_type, hacc_m, vacc_m, ve_mps, vn_mps, vu_mps, nsat
images.csv   t_s, cam, path   (path relativo alla cartella della sequenza)
truth.csv    t_s, e_m, n_m, u_m, lat_deg, lon_deg, alt_m, qw, qx, qy, qz   SOLO PER LA VALUTAZIONE
```

- Un solo orologio, `t_s` in secondi da 0.
- `t_replay.load(seq, cut_s=...)` toglie ogni riga GNSS con `t_s >= cut_s` e **non restituisce la verità**.
- La verità si legge solo con `load_truth(seq)`, da usare esclusivamente nel codice di valutazione.
- Verifica : `.venv/bin/python experiments/t_replay.py validate data/processed/t_replay/<sequenza>`.

### 4.2 Le sequenze

| Sequenza | Contenuto | Limiti | Per il VIO |
|---|---|---|---|
| **INSANE `mars_1`** (deserto del Negev) | Camera verso il basso 2056×1542 a 15 Hz con calibrazione K, IMU PX4 a 196 Hz, **barometro grezzo** a circa 18 Hz, GNSS non RTK a circa 5 Hz, verità RTK fissa | Volo basso e corto : circa 100 s, quota attorno a 5 m | **Primo candidato** : tutto sullo stesso orologio, barometro vero |
| **INSANE `outdoor_1`** (aerodromo di Klagenfurt) | Stessi sensori | 260 s ; RTK fisso solo il 22 % del tempo ; immagini 12,6 GB | Secondo test |
| **MUN-FRL**, campione `lighthouse` | Camera nadir 1440×1080 a 20 Hz, IMU a 400 Hz, RTK a 5 Hz, PPK separato | Niente barometro ; 181 s ; bag ROS di 3,8 GB | VIO senza barometro |
| Zurich `agz_1800_2400` | IMU 50 Hz filtrata + raw 10 Hz, barometro, GNSS, verità fotogrammetrica | Camera **in avanti** a livello strada ; timestamp delle immagini sbagliati da −1,9 a +0,3 s | Solo barometro |
| `wufeng_sim_base` / `wufeng_sim_low` | Camera resa dall'ortofoto reale 2020 di Taiwan, mappa 2018 ; IMU, barometro e GNSS SIMULATI | Suolo piatto, nessuna parallasse | Dimostrazione su Taiwan, etichetta SIMULATO |

### 4.3 Come rigenerare INSANE `mars_1`

Licenza : BSD-2 con condizioni aggiuntive, niente vendita, citare gli autori ; [pagina del dataset](https://cns-data.aau.at/insane-dataset/).

```bash
mkdir -p data/raw/insane/mars_1_images && cd data/raw/insane
curl -LO https://cns-data.aau.at/insane-dataset/mars_1_sensors.zip
cd mars_1_images
curl -LO https://cns-data.aau.at/insane-dataset/mars_1_nav_cam.zip
curl -LO https://cns-data.aau.at/insane-dataset/insane_sensor_calib_preprocessed.zip
unzip -q mars_1_nav_cam.zip && unzip -q insane_sensor_calib_preprocessed.zip
cd ../../../.. && .venv/bin/python experiments/t_export_insane.py --sequence mars_1
```

Controlla i percorsi attesi in `SEQUENCES` all'inizio di `experiments/t_export_insane.py`.

Attenzioni lette nel `meta.json` di INSANE :
- l'IMU è dichiarata ENU nel README del dataset ed è esportata senza rotazione : **verifica gli assi** prima di usarla, come hai fatto per Mid-Air, confrontando i rate del giroscopio con l'assetto della verità ;
- l'intestazione del file dei timestamp della camera dice `t[ns]`, ma i valori sono in secondi ;
- l'RTK è spostato di 1,606 s (`t_mag_gps`) per stare sull'orologio comune : l'exporter lo fa già.

### 4.4 MUN-FRL

- Campione già scaricato : `data/raw/mun_frl/lighthouse_francis_sample.bag` e `flight_dataset5_ppk.pos`. Licenza CC BY 4.0.
- Inventario : `.venv/bin/python experiments/v2_heldout_inventory.py`.
- Il bag non contiene i topic PPK/INS di posa : il file PPK separato dà solo la posizione. Il suo allineamento con i 181 s del bag non è stato stabilito.

## 5. Altri risultati della notte che toccano il VIO

1. **Il flusso ottico verso il basso non osserva la direzione.** L'avevi già visto ; lo confermiamo su ALTO, volo reale in elicottero.
   - Con una direzione perfetta, l'errore finale scende solo del 6 % ; con un'altezza dal suolo perfetta, del 30 % ; con le due insieme, del 39 % (SIMULATO sul percorso reale).
   - Un sensore solare (una fenditura su un sensore lineare, letteratura di Tsinghua, PUBBLICATO : [Wei et al. 2011](https://pmc.ncbi.nlm.nih.gov/articles/PMC3231287/)) ridurrebbe l'errore laterale da 196 a 31 m su ALTO, ma l'errore finale di appena il 6 %. Serve sole sotto i 70° e cielo sereno : circa il 40 % della giornata a Taichung in ottobre.
2. **La leva più grande su ALTO è la calibrazione della velocità della camera.** Con la conversione flusso → suolo nota su tutto il percorso (diagnostica che usa la verità), l'errore finale passa da 608 a 74 m. Stimarla in volo da coppie di fix successivi è fallito : un fix a ±15 m dà la scala solo a circa ±7 %.
3. **Stereo commerciale** : utile solo sotto i 30–90 m dal suolo. Con una base di 0,30 m a 60 m la disparità è di 1,3 px.
4. **Il filtro diventa troppo sicuro di sé quando sbaglia.** Lo vedi tu con la sola camera verso il basso, e Dustin nel navigatore ALTO : quando il fix è sbagliato, l'errore supera 3 σ nel 76 % dei frame. Il tuo `vio/evaluation/consistency.py` potrebbe diventare il controllo comune a tutti e due.
5. **Idee provate e scartate**, con criteri d'arresto scritti prima : bussola dalle ombre, matching sulle strade OpenStreetMap, campo magnetico (EMAG2 : circa 10 km di precisione), velocità da resistenza aerodinamica e vento sui log PX4. Non rifarle senza un'idea nuova.

## 6. Regole per non passare per « bullshit » davanti ai giurati

1. **La verità non entra mai nello stimatore dopo il taglio del GNSS.** Un sensore simulato si genera dalla verità solo dentro un generatore etichettato. Il tuo `vio/estimation/oracle.py` è già etichettato bene : continua così.
2. **Ogni numero porta un'etichetta** : MISURATO, SIMULATO o ORACLE.
3. **Un dataset mai visto per i numeri finali.** Dustin e questa notte hanno trovato che i 26–31 m di ALTO erano tarati sullo stesso tratto. Sul Train di ALTO (37,4 km, protocollo registrato prima, parametri congelati) la mediana per tratto è 94 m invece di 31, e il risultato si riproduce solo in 3 tratti su 8. Tu hai già tenuto da parte i voli di test : bene.
4. **Errore trovato da Dustin** : le immagini di riferimento di ALTO (`offset_0_None`) erano centrate sulla traiettoria vera, quindi ogni fix cadeva a meno di circa 48 m dal percorso vero. Lui l'ha corretto con una mappa unica, cercata in un cerchio attorno alla stima. Gli script ALTO di questo branch (`t_alto_heldout.py`, `r_alto_matchers.py`) usano ancora quelle immagini : i loro numeri sono da rifare.
5. **Mid-Air** : il giroscopio è espresso negli assi del mondo, al contrario della documentazione. Lo sai già ; vale per chiunque riusi il tuo codice.

## 7. Lavoro proposto, in ordine

| # | Compito | Criterio di riuscita | Tempo stimato |
|---|---|---|---|
| 1 | Sostituire il barometro simulato del tuo ESKF con il modello della sezione 3, 20 seed, rumore ×0,5 / ×1 / ×2 | Tabella dei 4 voli con mediana e p95 per livello di rumore, etichettata | 1–2 h |
| 2 | Far girare l'ESKF su INSANE `mars_1` : barometro vero, IMU vera, camera verso il basso, taglio del GNSS dopo 20 s | Grafici della traiettoria contro l'RTK e dell'errore nel tempo, IMU sola contro IMU + barometro + camera | 3–4 h (assi e orologi compresi) |
| 3 | Controllo di consistenza comune (NEES, frazione dei frame entro 3 σ) da applicare anche al navigatore ALTO di Dustin | Una funzione e un grafico usati da tutti e due | 1–2 h |
| 4 | Con Dustin : decidere quale baseline va su `main` | Un merge e una frase nel README | 30 min |
| 5 | Facoltativo : MUN-FRL senza barometro, per vedere il VIO su IMU a 400 Hz e camera nadir reale | Stessi grafici | 3 h |

Consiglio di consulenti esterni che abbiamo interpellato, Grok 4.7 e GPT-6 Astra (`docs/research/avis-externes.md`, in francese) : **congelare le funzionalità entro circa 12 ore** e dedicare il resto a figure, video e verifica. Per te vuol dire puntare prima sui compiti 1 e 2.

## 8. Mappa dei file di questo branch

- `docs/research/overnight-synthesis.md` : sintesi della notte e proposta classificata (francese).
- `docs/research/sensor-fusion.md` : barometro, IMU, stereo, sensore solare, budget di deriva (francese).
- `docs/research/datasets-replay-sim.md` e `docs/research/data-manifest.md` : tutti i dataset valutati, con licenze, dimensioni e stato ; formato di replay ; patch proposta per il simulatore Gazebo (francese).
- `docs/research/map-localization.md` : matching camera → mappa (francese).
- `docs/research/avis-externes.md` : pareri di Grok 4.7 e GPT-6 Astra (francese).
- `questions.md` : domande aperte per il team.
- `experiments/` : script con i prefissi `n_` a `v`. Per te :
  - `n_sensor_fusion.py` : simulazione verticale e direzione ;
  - `p_zurich_baro.py`, `s_zurich_vertical.py`, `s_insane_baro.py`, `s_insane_vertical.py`, `s_px4_rtk_baro.py` : barometro reale ;
  - `t_replay.py`, `t_export_insane.py`, `t_export_zurich.py`, `t_gen_wufeng_replay.py` : formato di replay ;
  - `v2_heldout_inventory.py` : MUN-FRL e OrthoLoC.
- Simulatore Gazebo (branch `simulations` di Dan) : i sensori funzionano in tempo reale su un Mac, ma il topic `/imu/data` pubblicava l'**assetto vero**. Correzione proposta in `docs/research/sim_patch.diff` (taglio GNSS, registratore, seconda camera, fuga d'assetto), descritta in `datasets-replay-sim.md`. Non è applicata al branch di Dan.
