# Challenge 2 in depth: navigation without GPS

Research on what already exists for navigation when satellite positioning is jammed, where it falls short, and what a team of six could add in a weekend. Read [challenge-decision.md](challenge-decision.md) first for the comparison with the other challenges.

## Short version

- Navigation by camera is a fielded capability. Commercial and Ukrainian systems match what a drone's camera sees against stored maps and report accuracy of around 20 metres.
- Its known weak points are water and featureless ground, night and bad light, maps that are out of date, and the computing power it needs.
- Taiwan's version of the problem is harder than the land case: GPS interference around the outlying islands is reported regularly, and much of the route is over water, where map matching has nothing to match.
- A weekend entry cannot beat the fielded systems on land. It can show a cheap layered approach that keeps working, with honest uncertainty, when one layer drops out, including a water crossing.
- The data question is solved: free aerial images of one Taiwanese site from two different dates are available, so we can fly a simulated drone over one and use the other as the on-board map.

## What the brief asks for

Use IMU, speed, heading, ranging, landmark or visual information to limit drift without satellite positioning. Build a dead-reckoning baseline, add at least one correction, plot the estimated path against the reference and the error over time, and explain the limits when sensors fail or noise rises. Target platform, hardware needs and cost are scored.

## Existing products and fielded systems

| System | What it does | Source |
|---|---|---|
| UAV Navigation VNS01 | Visual navigation unit for drones. Updated in September 2026 with terrain-referenced navigation and satellite map matching | [GPS World](https://www.gpsworld.com/uav-navigation-enhances-vns01-for-precision-drone-navigation-in-gnss-denied-environments/), [product page](https://www.uavnavigation.com/gnss-denied-navigation) |
| OSCAR (Ukraine) | Matches live camera imagery with mapped landmarks. Reported accuracy within about 20 metres, in fog and at night | [The Defense Post](https://thedefensepost.com/2026/01/29/ukraine-drones-vision-navigation/) |
| Polaris SkyPASS | Sky sensor that images the sun or moon, the polarisation pattern of the sky, and stars, and returns an absolute heading. The maker claims better than 0.1 degrees | [Polaris Sensor Technologies](https://www.polarissensor.com/skypass/) |
| PX4-Flow | Optical-flow sensor for drones: a downward camera plus a range sensor that outputs speed over ground | [firmware on GitHub](https://github.com/PX4/PX4-Flow) |
| WildNav (open source) | Locates a drone by matching its photos against satellite images, built for non-urban terrain | [TIERS/wildnav](https://github.com/TIERS/wildnav) |

Two patterns stand out. Fielded systems combine a relative source (inertial sensors, optical flow) that drifts with an absolute source (map matching) that does not. And the sun and sky are used commercially as a heading reference, which supports the advice we got from the mentor.

Coverage of the war in Ukraine also reports small camera modules for the last part of a flight that cost under a hundred dollars per unit. I have not checked the primary source for that figure, so verify it before using it in a pitch.

## Known limitations

- **Water and featureless ground.** Map matching needs distinctive features. Over open water, desert or snow there are few, and waves create false ones ([review in Satellite Navigation](https://link.springer.com/article/10.1186/s43020-025-00162-z)).
- **Night and light.** Cameras depend on illumination. Thermal cameras help at a higher cost.
- **Maps that have changed.** Seasons, construction and floods make the stored map differ from what the camera sees ([season-invariant localisation](https://arxiv.org/pdf/2110.01967)).
- **Computing power.** Matching live images against maps takes compute and storage that small, cheap vehicles lack ([IEEE Spectrum](https://spectrum.ieee.org/drone-gps-alternatives)).
- **Altitude.** Optical flow gives speed only when the height above ground is known. An error in height becomes the same percentage error in speed.
- **Drift between fixes.** Every relative source accumulates error, so the question is always how long the system survives without an absolute fix.
- **Sun and sky.** A sun compass needs the time and a view of the sky. Under heavy cloud the sun's disc is gone, although the polarisation pattern of the sky can still give a heading ([Royal Society Interface](https://royalsocietypublishing.org/rsif/article/16/150/20180878/87000/Polarized-skylight-based-heading-measurements-a)).

## The situation in Taiwan

- Taiwan is regularly subjected to GPS interference, and in a dense maritime theatre like the Strait it is hard to isolate and avoid ([Breaking Defense, January 2026](https://breakingdefense.com/2026/01/china-gps-advantage-taiwan-altpnt/)).
- In June 2025, open-source observers reported severe jamming along China's coast that affected nearly all of Taiwan's outlying islands during naval exercises ([post on X](https://x.com/OSINTWarfare/status/1939004535805968441)). This is a social-media source; look for a confirmed report before quoting it.
- The hackathon's own description names "GNSS-denied navigation across the Strait". The Strait is roughly 130 km wide at its narrowest, so a vehicle crossing it flies or sails over water for most of the route.

This is what makes the local problem different from Ukraine's. Over land, map matching carries the load. Over water it cannot, and the vehicle must hold a heading and estimate speed by other means until it reaches a coastline.

## Where a weekend entry can add something

1. **A layered estimate with honest uncertainty.** Speed from optical flow, heading from a sun compass, position fixes from map matching, all combined in a particle filter that reports how sure it is. Show which layer carries the estimate in which conditions.
2. **A water-crossing scenario.** Take away map matching for a stretch, let the uncertainty grow, then recover at the coastline with one position fix. No fielded product solves this, and it is the Taiwanese case.
3. **Cold start.** Find heading and a first position with no GPS at all, from the sun and one map match.
4. **A map from a different date.** Use an older image as the on-board map and a newer one as the world the drone sees, so the result does not depend on matching an image against itself.
5. **A compute budget.** Measure how long each step takes on a plain CPU and state what hardware the method needs.

Ideas 1, 2 and 4 form one coherent entry. Ideas 3 and 5 are short additions.

## Data

- **Aerial images of Taiwan** (`data/raw/aerial`, licence CC BY 4.0, from [OpenAerialMap](https://openaerialmap.org/)): two images of the same site in Wufeng, Taichung, about 2.1 by 2.9 km each. I checked that both use the same map projection and cover almost exactly the same area.
  - 3 May 2018, original resolution about 4.9 cm per pixel, 115 MB
  - 23 March 2020, original resolution about 3.6 cm per pixel, 385 MB
  - `python scripts/fetch_aerial.py` fetches both at a quarter of the resolution (about 20 and 14 cm per pixel), which is enough for a simulated flight at around 100 m and avoids the full download.
  - A third image of the same site from September 2020 is also available.
  - Both are fetched and checked: they open, line up, and show the same ground with visibly different colours and crops. Each covers a corridor along a motorway, about 3 km long and 300 to 600 m wide, so only a quarter to a third of the rectangle holds imagery. A simulated flight has to follow the corridor.
  - OpenAerialMap lists 371 images for Taiwan, 84 of them freely licensed and large enough for a simulated flight.
- **How we use them:** fly a virtual drone over the 2020 image on a known path and cut out what a downward camera would see. That gives frames, altitude and an exact true path. The 2018 image is the on-board map.
- **Real flight data for validation:** [UAV-VisLoc](https://github.com/IntelliSensing/UAV-VisLoc) has 6,742 drone images with position, height and heading, and 11 satellite maps. The full set is 16.4 GB; a sample is 2.04 GB, hosted on Google Drive. The repository states no licence.
- **Organiser dataset:** the brief mentions IMU, speed, heading and reference position. Not seen yet.
- **Phone recording:** walk a loop with the phone camera pointing down and use the phone's GPS only as the reference.

## Tools

- OpenCV for optical flow and for feature matching (installed)
- [LightGlue](https://github.com/cvg/LightGlue), a modern image matcher with an Apache-2.0 licence, if plain feature matching is not robust enough across the two dates
- [filterpy](https://github.com/rlabbe/filterpy) for the Kalman filter (installed), and the free book [Kalman and Bayesian Filters in Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)
- rasterio for reading large aerial images (installed)
- A sun-position library such as `astral` or `pvlib` for the sun compass (to be added)
- [WildNav](https://github.com/TIERS/wildnav) as a reference implementation. Check the licence of the matcher it depends on before reusing code.

## Reading list

Five papers, one per building block. Selected from their abstracts; read the relevant one before building your part.

1. **Map matching and cold start.** Kinnari, Verdoja, Kyrki (2021), "GNSS-denied geolocalization of UAVs by visual matching of onboard camera images with orthophotos", ICAR 2021. [arXiv](https://arxiv.org/abs/2103.14381). Locates a drone from inertial data, one camera and an aerial map, using a particle filter that starts from a rough guess. Closest to our overall design.
2. **Maps from a different season.** Kinnari, Verdoja, Kyrki (2022), "Season-invariant GNSS-denied visual localization for UAVs", IEEE Robotics and Automation Letters. [arXiv](https://arxiv.org/abs/2110.01967). Matching camera images to a map taken in another season. Directly relevant to using the 2018 image as the map for a 2020 flight.
3. **Speed from optical flow.** Honegger, Meier, Tanskanen, Pollefeys (2013), "An open source and open hardware embedded metric optical flow CMOS camera for indoor and outdoor applications", ICRA 2013. [Semantic Scholar](https://www.semanticscholar.org/paper/An-open-source-and-open-hardware-embedded-metric-Honegger-Meier/579e2e12c1e46e8636206eb5028ecf04bf5c205c). The PX4-Flow sensor: optical flow scaled by distance to the ground and corrected for rotation with a gyroscope. This is the method the mentor described, on real hardware.
4. **Heading from the sky.** Dupeyroux, Viollet, Serres (2019), "Polarized skylight-based heading measurements: a bio-inspired approach", Journal of the Royal Society Interface. [open access](https://pmc.ncbi.nlm.nih.gov/articles/PMC6364636/). A two-pixel sensor modelled on desert ants that reads heading from the polarisation of the sky. Shows how cheap a sky compass can be.
5. **Overview and limits.** "GNSS-denied unmanned aerial vehicle navigation: analyzing computational complexity, sensor fusion, and localization methodologies" (2025), Satellite Navigation. [Springer](https://link.springer.com/article/10.1186/s43020-025-00162-z). A review of methods, their computing cost and where they fail. Useful for the pitch and the limits slide.

With code: the WildNav paper, "Vision-Based GNSS-Free Localization for UAVs in the Wild" ([repository](https://github.com/TIERS/wildnav)).

## What the papers tell us

Papers 1, 2 and 3 were read in full. Paper 5 was read through a partial extract. Paper 4 could not be opened from here (the sites block automated access), so open it in a browser before relying on it.

### Findings

- **Heading is the bottleneck.** In both Kinnari papers the filter starts with no heading information, first diverges, and needs about 2 km of flight to converge. The optical-flow paper reports that outdoor drift came mainly from compass errors. A sky compass at the start removes exactly this problem, which is what the mentor proposed.
- **Expect tens of metres.** Paper 1 reports about 50 m RMS error after convergence and under 20 m over areas with roads, against 217 to 253 m for odometry alone over 4 to 6 km. Paper 2 reports 26.5, 29.1 and 30.6 m on three real flights after 2 km. Both used maps at 1 m per pixel from about 92 m altitude.
- **The published method is a particle filter.** Both papers use Monte-Carlo localisation with 1000 particles over position, heading and a scale factor. Map matching is ambiguous (neighbouring fields and road junctions look alike), and a filter that tracks many hypotheses survives that where a single estimate gets lost.
- **Featureless stretches are handled by falling back.** Paper 2 describes a flight over a lake: the matching score stays flat, all nearby particles keep similar weight, and the estimate relies on odometry until distinct ground returns.
- **Our simulation plan is a published protocol.** Paper 2 runs 100 simulated flights with one aerial image as the map and another image of the same area, from a different date, as what the drone sees. It adds noise of 2 m per 100 m step in position and 1 degree in heading. Paper 1 simulates its inertial data from the true path plus noise.
- **Simple matching works when the two images look similar.** Paper 1 compares twelve classical measures and finds two that separate right from wrong positions best (Moravec and zero-normalised cross-correlation). Paper 2 shows these fail under winter against summer and that a trained network does better. Our two images differ in crops and colour, with no snow, which is the milder case.
- **Optical flow gives metric speed with one formula.** Speed equals image flow times distance to the ground divided by focal length, after subtracting the rotation measured by a gyroscope. The sensor runs at 250 Hz on a microcontroller, draws 0.575 W and measures 45 by 35 mm. Integrated over a 28 m indoor loop it closed to within 0.25 m.
- **Its range sensor does not reach flight altitude.** The sensor uses ultrasound, which works for a few metres. At 100 m a laser rangefinder or a barometer with a terrain model is needed, and any height error becomes a speed error of the same percentage. The scale factor in the particle filter absorbs this.
- **Matching is cheap enough.** Paper 2 needs 0.33 s plus 0.13 s per update for 1000 particles on a laptop, with one update every 100 m of flight.
- **The review does not cover sky compasses.** In the extract I could read, celestial and polarisation navigation are not mentioned, and finding the initial position is listed as an open challenge.

### Consequences for our design

1. Use a particle filter over position, heading and scale, as in papers 1 and 2. It is also simpler to write than a Kalman filter.
2. Match by template at 1 m per pixel with zero-normalised cross-correlation. With heading known from the sky compass, no rotation search is needed, which cuts the compute.
3. Copy the simulation protocol and noise values of paper 2 and cite it.
4. Make the headline experiment the one the papers leave open: distance to convergence and error with and without a sky-compass heading.
5. Second experiment: a stretch with matching switched off, showing error growth and recovery.
6. Promise tens of metres, and compare against dead reckoning drifting to hundreds.

### What this means for originality

Map matching with odometry in a particle filter is established work from 2021 and 2022, funded by Saab. The fallback over featureless ground is described there in one paragraph. What we would add is the sky-compass heading for cold start and cheaper matching, measured, and a quantified long stretch without fixes, on Taiwanese imagery.

## Earlier entries

- Rome, third place: a visual positioning kit that matches camera images against Earth-observation maps. The team is developing it commercially ([recap](https://eurodefense.tech/back-in-rome-recap-of-our-second-european-defense-tech-hackathon-in-italy/)).
- An earlier winning team had a group of drones share observations to localise without GPS, as reported by [DroneXL](https://dronexl.co/2024/12/03/europes-largest-defense-tech-hackathon-drone-detection/).

Map matching on its own has therefore been shown at these events already. The layered estimate and the water crossing are what would be new.

## Six roles

1. Flight simulator: frames, altitude, true path and sensor noise from the 2020 image
2. Optical flow: speed over ground from the frames
3. Sun compass and cold start: heading from time and sun direction
4. Particle filter: combine the sources over position, heading and scale, and report uncertainty
5. Map matching: position fixes against the 2018 image
6. Visualisation, evaluation, hardware concept and pitch

## Demo

1. A map shows the true path and the dead-reckoning estimate drifting away.
2. Camera speed and sun heading are switched on. The drift slows.
3. Map matching is switched on. The estimate snaps back to the true path at each fix.
4. The drone crosses a stretch with no usable features. The uncertainty ellipse grows, then collapses at the first fix on the far side.
5. One chart: position error over time for all three configurations, with one headline number.
6. One slide: sensor package, compute and cost per drone.

## Risks

- This option has the most parts. If the simulator is late, everything waits. Build it first.
- Matching across two dates may fail where the site changed. That is also a finding worth showing.
- A simulated flight over a flat image ignores terrain height, camera tilt and motion blur. Say so, and add noise for the ones that matter.
- Judges may ask for real flight data. The UAV-VisLoc sample or a phone recording answers that.
- The site is inland, so the water stretch is simulated by masking the map. A coastal image would be more convincing if one of suitable size exists.

## Open question for the mentor

He named the position of the sun and one more input for the cold start. Candidates are an accurate clock, the polarisation pattern of the sky, a magnetometer, or a single map match. Ask which he meant.

## How it compares with Challenges 7 and 4

| | Challenge 2 | Challenge 7 | Challenge 4 |
|---|---|---|---|
| Data today | Ready: two aerial images, simulation needed | Ready | Ready at clip level |
| Local relevance | High: interference reported, water crossing | Medium | Highest: cancelled contract |
| Fielded products | Yes, on land | Yes | Yes |
| Gap we can show | Layered estimate, water crossing | Quiet contacts, evidence | Bird rejection by movement |
| Record at earlier events | Placed | First and second place | First place, most entries |
| Competition expected | Low | Medium | High |
| Mentor match | Strong | Strong | Partial |
| Main risk | Many parts | Routine without the extras | Detector quality |
