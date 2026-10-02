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

1. **A layered estimate with honest uncertainty.** Speed from optical flow, heading from a sun compass, position fixes from map matching, all combined in a Kalman filter that reports how sure it is. Show which layer carries the estimate in which conditions.
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

## Earlier entries

- Rome, third place: a visual positioning kit that matches camera images against Earth-observation maps. The team is developing it commercially ([recap](https://eurodefense.tech/back-in-rome-recap-of-our-second-european-defense-tech-hackathon-in-italy/)).
- An earlier winning team had a group of drones share observations to localise without GPS, as reported by [DroneXL](https://dronexl.co/2024/12/03/europes-largest-defense-tech-hackathon-drone-detection/).

Map matching on its own has therefore been shown at these events already. The layered estimate and the water crossing are what would be new.

## Six roles

1. Flight simulator: frames, altitude, true path and sensor noise from the 2020 image
2. Optical flow: speed over ground from the frames
3. Sun compass and cold start: heading from time and sun direction
4. Kalman filter: combine the sources and report uncertainty
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
