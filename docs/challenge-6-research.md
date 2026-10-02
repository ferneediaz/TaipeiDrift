# Challenge 6 in depth: maritime anomaly alerting

Research on what already exists for spotting unusual vessel behaviour, where it falls short, and what a team of six could add in a weekend. Read [challenge-decision.md](challenge-decision.md) first for the comparison with the other challenges.

## Short version

- Maritime anomaly detection is a commercial product category. Platforms score vessel behaviour, flag vessels that stop reporting, and route alerts to analysts.
- The known weak points are that the position reports (AIS) can be switched off or falsified, that confirmed examples of bad behaviour are rare, and that fishing vessels cause many false alarms.
- Taiwan has a concrete, recent problem: undersea cables damaged by vessels that loitered over them and used false identities.
- A weekend entry cannot match the global platforms. It can show a focused cable-protection watch: loitering, reporting gaps and identity changes near cable zones, each alert explained, with a measured comparison of rules against a learned method.
- This is the lowest-risk build of the candidates. It needs no camera data, no model training on video and no hardware.

## What the brief asks for

A dashboard on public, anonymised or simulated AIS tracks that detects stopping, deviation, clustering, zone entry and reporting gaps, gives each alert a risk score, a timeline and an explanation, lets an operator mark false alarms and adjust thresholds, and compares a rule-based method with a statistical or machine-learning method. Treatment of incomplete data and of uncertainty is scored.

AIS is the position broadcast that larger vessels are required to send: identity, position, speed and heading every few seconds to minutes.

## Existing products

| Product | What it does | Source |
|---|---|---|
| Windward | AI models that surface anomalies in vessel behaviour, and matching of satellite detections to vessel identities to find vessels that are not reporting | [Windward](https://windward.ai/solutions/mda/) |
| Starboard Maritime Intelligence | Detects anomalous behaviour, scores risk and routes alerts into analyst workflows | [Starboard](https://www.starboardintelligence.com/) |
| Global Fishing Watch | Open platform showing vessel presence worldwide from AIS since 2012, with an API | [data availability](https://globalfishingwatch.org/global-fishing-watch-data-availability/) |

The commercial platforms combine AIS with satellite imagery and radio-frequency sensing, because AIS alone can be switched off.

## Known limitations

- **AIS is voluntary in practice.** A vessel can switch it off or broadcast a false identity. The vessel suspected in the January 2025 cable incident had used six different AIS numbers and at least two names in six months ([The War Zone](https://www.twz.com/news-features/taiwan-coast-guard-blames-chinese-owned-ship-for-cutting-undersea-communications-cable)).
- **Gaps are ambiguous.** Fishing vessels often switch AIS off to hide fishing grounds, and coverage has holes. A reporting gap alone is weak evidence.
- **Confirmed cases are rare.** Research mostly tests on injected synthetic anomalies because real labelled events are hard to obtain ([review of AIS anomaly detection](https://www.researchgate.net/publication/357833953_Anomaly_Detection_in_Maritime_AIS_Tracks_A_Review_of_Recent_Approaches)).
- **False alarms.** In dense traffic, unusual is common. The brief itself asks that not every anomaly be treated as a threat.
- **Raw data is commercial.** Global historical AIS is sold, so free raw tracks for the Taiwan Strait are hard to get ([Global Fishing Watch FAQ](https://globalfishingwatch.org/faqs/can-i-download-raw-ais-data/)).

## The situation in Taiwan

- **January 2025:** a cable north of Taiwan was damaged. The coast guard linked it to the Cameroon-flagged Shunxing 39, owned by a Hong Kong company, which could not be boarded because of weather ([The War Zone](https://www.twz.com/news-features/taiwan-coast-guard-blames-chinese-owned-ship-for-cutting-undersea-communications-cable)).
- **February 2025:** the coast guard detained the Hong Tai 58 after a cable to Penghu was cut. It had loitered within about 925 metres of the cable, and the name it broadcast differed from the name on its hull ([Newsweek](https://www.newsweek.com/taiwan-seizes-china-owned-ship-undersea-cable-sabotage-2036517)).
- **April 2026:** a further case of suspected cable damage by a Chinese vessel was reported ([Taipei Times](https://www.taipeitimes.com/News/taiwan/archives/2026/04/02/2003854889)).

The February case is almost a textbook example of the behaviours this brief lists: loitering near a protected asset, and an identity that does not match.

## Where a weekend entry can add something

1. **A cable-protection watch.** Zones around cable routes, with alerts for loitering, slow passes and anchoring inside them. Narrow and concrete, instead of a general anomaly dashboard.
2. **Identity consistency.** Flag vessels whose broadcast name, number or type changes over time. This was the tell in both 2025 cases and is simple to compute.
3. **Explained, uncertain alerts.** Each alert states which behaviours triggered it and how confident the system is, for example whether a gap falls in an area of known poor coverage.
4. **A measured comparison.** Rules against a learned method (such as an isolation forest) on simulated tracks with labelled events, reported as precision, recall and false alarms per day.
5. **Operator feedback that changes the system.** Marking a false alarm or moving a threshold updates the alert list live.

## Data

- **Simulated tracks with labelled events.** The brief allows this, and it gives the ground truth needed to report detection performance. Normal traffic follows lanes between ports; injected events are loitering, gaps, deviation and identity changes.
- **Live AIS.** [aisstream.io](https://aisstream.io/) offers a free live feed by map area, with a sign-up for an API key. Recording the waters around Taiwan during the weekend would give real tracks for the demo. I have not checked how good its coverage of the Taiwan Strait is.
- **Historical AIS for other regions:** [MarineCadastre](https://hub.marinecadastre.gov/) for US waters.
- **Cable routes:** public cable maps exist; approximate routes are enough for a prototype and should be labelled as approximate.

## Tools

- pandas and scikit-learn for features, rules and the learned method (installed)
- FastAPI for the service (installed) and a web map for the dashboard
- No model training on images or audio, and no large downloads

## Earlier entries

- [EDTH-ctrl_sea](https://github.com/bb1/EDTH-ctrl_sea): AIS and radar input, a matching step, a risk engine and a map.
- Copenhagen 2025, third place: maritime, but hardware (a remotely controlled vessel).

The record at earlier events is thin, which can mean an open field or a topic judges at those events cared less about.

## Six roles

1. Track simulator with labelled events
2. Rules: loitering, zone entry, gaps, deviation, identity changes
3. Learned method and the comparison
4. Map dashboard: tracks, zones, alert cards, timeline
5. Operator feedback, thresholds, live AIS recording
6. Operator and customer concept, data and service model, pitch

## Demo

1. A map of the waters around Taiwan with cable zones and vessel tracks replaying.
2. A vessel slows and loiters inside a cable zone. An alert appears with the reason, a risk score and a timeline.
3. The same vessel's broadcast name changes. The risk score rises.
4. A second vessel stops reporting in an area of known poor coverage. It is flagged with low confidence.
5. The operator marks a false alarm and moves a threshold. The list updates.
6. One chart: rules against the learned method, on precision, recall and false alarms per day.

## Risks

- A general "anomaly dashboard" looks like every commercial product. The cable focus is what makes it specific.
- Results on simulated events can be questioned. Model the simulated events on the documented 2025 cases and say so.
- Live AIS coverage may be poor. Treat it as an extra, not as the base.

## How it compares

| | Challenge 2 | Challenge 4 | Challenge 6 | Challenge 7 |
|---|---|---|---|---|
| Build risk | Highest | Medium | Lowest | Low |
| Data today | Aerial images ready, flight to be simulated | Clips ready | Simulate, live feed possible | Ready |
| Local story | GPS interference, water crossing | Cancelled counter-drone contract | Cable incidents | General |
| Originality of our angle | High | Medium | Medium | Medium |
| Record at earlier events | Placed | First place | Thin | First and second |
| Mentor match | Strong | Partial, with stated interest | Good (maritime) | Strong |
| Demo | Map animation | Video with tracks | Map with alerts | Live microphone |
