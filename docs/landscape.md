# Landscape: existing products, their limits, and what we should build

Research on Friday evening, after the first experiments in [experiments.md](experiments.md). It answers three questions: what exists for navigation without GNSS, where it falls short, and what a team of six can add in a weekend.

## Short version

- The high end is solved. Laser velocity sensors, star trackers and magnetic-anomaly navigation hold position over land and sea, on aircraft that can carry and afford them.
- Taiwan is already adopting a foreign product. Maxar's Raptor is being rolled out to Taiwan's drone industry through AIDC. It needs Maxar's own 3D terrain data.
- Our core idea is proven in combat. Ukraine's special forces have flown it at scale since 2023 ("Eagle Eyes"), and the Tomahawk missile used it in the 1980s. See [the Eagle Eyes section](#eagle-eyes-and-the-systems-fielded-in-ukraine), added Saturday.
- Open-source autopilots already estimate the wind and use it for dead reckoning when GNSS is lost. Our wind result is therefore known practice, with one catch: they learn the wind while GNSS still works.
- The gaps we can address: cheap sensors with open maps, a system that knows when it is wrong, and a way to tell before the mission where navigation will hold.
- Proposal: build the navigator with open data and cheap sensors, add an integrity check, and add a map of Taiwan that shows the expected position error along a planned route.

## Existing products

| Product | How it works | Reported performance | Limits for our case |
|---|---|---|---|
| [Maxar Raptor](https://vantor.com/product/mission-solutions/raptor/) | Ordinary camera matched against Maxar's Precision3D terrain data | Under 10 m RMSE, works at night and at low altitude | Needs Maxar's proprietary data. Nothing to match over open water |
| [UAV Navigation VNS01](https://www.gpsworld.com/uav-navigation-enhances-vns01-for-precision-drone-navigation-in-gnss-denied-environments/) | Visual navigation plus terrain-referenced navigation and satellite map matching | Not stated in the sources I read | A dedicated hardware unit |
| [OSCAR](https://thedefensepost.com/2026/01/29/ukraine-drones-vision-navigation/) (Twist Robotics, Ukraine) | Camera matched against mapped landmarks | About 20 m, in fog and at night | Land only |
| [Eagle Eyes](https://www.uasvision.com/2024/06/04/ukraines-special-forces-have-drones-that-fly-without-gps/) (Ukraine's special forces) | Live video of the ground compared with an on-board map stitched from earlier reconnaissance photos and video; also recognises targets | No accuracy published. In wide use on one-way attack drones since 2024 | Needs a recent reconnaissance flight over the route |
| [Osiris](https://www.delian.ai/osiris) (Delian Alliance Industries, Greece) | Cameras, inertial sensing and preloaded satellite imagery | Maker: 15 m CEP by day; under 20 m on average over about 3,000 km of trial flights in Ukraine | Maker's figures. Under 300 g, 25 W |
| [AIDC AIxVNAV](https://thedefensepost.com/2026/06/03/taiwan-drones-without-gps/) (Taiwan) | Low-cost camera against satellite-derived 3D imagery, with Vantor's Raptor Guide and ACE software under licence | Flight-tested in 2026; "centimetre-level target positioning", which is the position of a target, not of the drone | Licensed foreign data and software |
| [Advanced Navigation LVS](https://www.advancednavigation.com/tech-articles/laser-velocity-sensor-lvs-high-accuracy-velocity-aid-gnss-denied-navigation/) | Infrared lasers measure speed over ground or sea by Doppler shift | 0.045% of distance over a 545 km flight, with a tactical-grade inertial unit | Price and weight not found. Tested with high-grade inertial hardware. It emits |
| [Honeywell celestial navigation](https://aerospace.honeywell.com/us/en/products-and-services/products/navigation-and-sensors/navigation-systems/celestial-aided-navigation) | Star tracker measuring stars and satellites | Works over oceans | Aircraft-class equipment, needs a view of the sky |
| [Honeywell magnetic-anomaly navigation](https://www.honeywellaerospace.com/us/en/products-and-services/products/navigation-and-sensors/navigation-systems/magnetic-anomaly-aided-navigation) and similar | Sensitive magnetometer matched against a magnetic map | Works over water | Needs magnetic maps and a magnetically quiet aircraft |
| [Bavovna](https://bavovna.ai/) | Inertial sensors, compass, barometer and airflow sensor, corrected by a trained model | Maker reports a 250 km flight without GNSS | No map, so errors still accumulate. The accuracy claim is the maker's own |
| [PX4](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf) and [ArduPilot](https://ardupilot.org/dev/docs/ekf2-estimation-system.html) autopilots | Estimate wind from airspeed and use it to dead-reckon when GNSS is lost | Depends on how much the wind changes | The wind is learned while GNSS still works |
| [Low-cost radar terrain navigation](https://www.mdpi.com/2673-4591/88/1/11) (research, 2025) | Automotive radar and barometer matched against an elevation map | Tested over flat and mountainous terrain | Research prototype. Flat terrain remains weak |

## What this means for Taiwan

- Maxar is partnering with Taiwan's Aerospace Industrial Development Corporation to deploy Raptor across the Taiwanese drone industry ([GPS World](https://www.gpsworld.com/maxar-helps-accelerate-the-resilience-of-taiwans-uav-industry-against-gps-interference/), [SpaceNews](https://spacenews.com/taiwanese-aerospace-firm-partners-with-maxar-on-gps-alternative-drone-navigation/)). Judges from Taiwan's drone community will probably know it.
- That confirms the need is real and funded. It also means "camera matched against a map" is not new here.
- Raptor depends on one supplier's data. A method that runs on open or Taiwanese map data is a different offer.

## Where existing solutions fall short

1. **Cost and size.** The systems that hold position over water are built for aircraft. Small and expendable drones cannot carry or afford them.
2. **Dependence on proprietary maps.** The leading camera product needs its vendor's terrain data.
3. **Open water with cheap sensors.** Affordable systems rely on ground features. Over the sea they fall back on dead reckoning.
4. **Wind learning needs GNSS.** Autopilots learn the wind before GNSS is lost. A drone that is jammed from take-off never learns it, unless another source of position fixes takes that role.
5. **Knowing when the estimate is wrong.** In our tests a blind camera match was wrong in 14 to 30 percent of cases, and our first terrain filter reported 7 m of uncertainty while it was 1.2 km off. Product descriptions say little about this.
6. **Knowing in advance where it will work.** Every result in our experiments depended on the ground below. An operator planning a route has no tool that says where the navigation will hold.

## How each can be addressed in a weekend

| Gap | Our answer | Evidence we already have |
|---|---|---|
| Cost and size | Camera, single-beam laser or cheap radar, airspeed sensor | Terrain matching held 9 m in mountains with a 4 m sensor error |
| Proprietary maps | Copernicus elevation and OpenAerialMap imagery, both free | Both loaded and used in the experiments |
| Open water | Wind learned from camera and terrain fixes, then carried across | 552 m after 30 km, against 2,702 m without |
| Wind learning needs GNSS | Camera and terrain fixes replace GNSS as the source the wind is learned from | Same experiment |
| Knowing when it is wrong | Check every fix against the filter before accepting it, and compare sensors with each other | Camera fixes were 99% right when checked against a prior |
| Knowing in advance | A map of expected position error, computed from elevation, imagery and water | 44% of land in our strip is too flat for terrain matching |

## What we should build

This was the proposal on Friday evening. The current plan is in [PLAN.md](PLAN.md). It keeps the navigator and the integrity check, and it replaces the map-based parts, because the team chose the Mid-Air dataset, which has no aerial map.

One navigator and one planning view, both on open data.

### 1. The navigator (meets every required deliverable)

- Dead-reckoning baseline from airspeed and heading.
- A particle filter over position and wind.
- Terrain fixes from a downward range sensor and the open elevation model.
- Camera fixes from the open aerial imagery, accepted only when they agree with the filter.
- Sun compass for the cold start.
- Plots of true path, estimates and error over time, for each configuration.

### 2. The integrity check (the limits deliverable)

- Every fix is tested against the filter's current belief before it is used.
- The system reports when its sensors disagree or when it has had no fix for too long.
- Demo: show the failure we hit ourselves, a filter that is confident and wrong, and show it being caught.

### 3. The navigability map (what sets the entry apart)

- A map of Taiwan and the Strait coloured by expected position error: terrain fixes in the hills, camera fixes on the plain, growing drift over water.
- For a planned route, the predicted error along the way and at arrival.
- This is a planning tool for the operator, and it uses the same error model as the navigator.

### What we would claim

A navigation method for small drones that runs on open maps and cheap sensors, tells the operator before launch where it will hold, and reports during flight when it should not be trusted.

### What we would not claim

- Better accuracy than Raptor or the laser velocity sensor.
- A solution for the full Strait crossing. We have simulated 30 km.
- Performance at night or in fog. Not tested.

## How Raptor and VNS01 work, and what they publish about night

Researched on Friday night from the vendors' developer pages and datasheets, two test reports by Inertial Labs (a partner of Vantor), a technical paper by UAV Navigation and two patents. We found no independent test of either product. The key quotes were checked against saved copies of the sources.

### Raptor Guide (Vantor, formerly Maxar)

- **What it does with one camera frame.** It takes the frame and a rough position and attitude from the drone's own navigation, renders how the 3D map would look from there, matches the frame against that rendering, and computes the camera's position and attitude. It returns a confidence between 0 and 1 and an uncertainty. Source: [integration guide](https://developers.maxar.com/raptor/guide/1.0/sdk/integration).
- **It has no memory.** It "performs separate camera pose estimations for each individual frame" and "should not be used as a standalone navigation system". It is meant to be fused with an IMU and other sensors.
- **Search.** Given an uncertainty for the rough position, it searches a region around it. Without one, it only refines that single guess.
- **Rejection.** The guide rates a confidence below 0.7 as "Poor — reject; no robust match found".
- **Reference data.** A textured 3D surface model at 50 cm, derived from 30 cm satellite imagery. Source: [terrain datasheet](https://cdn.sanity.io/files/ava0h2e5/production/5a74a29c81f5f8a6213542a1c674c9a687dfa93e.pdf).
- **Camera.** A visible-light or infrared video feed. Wide lenses are not recommended and fisheye lenses are not supported.
- **Computing.** A graphics processor is required. There is a build for the Jetson Orin Nano, and the guide asks for at least 4 GB of memory.
- **Stated limits.** The match is poor over water and sky, where the terrain has changed since mapping, and at low contrast. Source: [troubleshooting page](https://developers.maxar.com/raptor/guide/1.0/sdk/raptor-guide-troubleshooting).
- **Published test.** Eight daytime flights on a Cessna 182 with a fixed infrared camera, at 62 m/s and 100 to 1,000 m above ground, fused with an inertial system. The mean error is 12.2 m, half of the positions are within 9.3 m and 95 percent within 25.8 m. The best flight, at 7.0 m, had the camera mounted at 45 degrees. Segments over large water were removed. Source: [Inertial Labs report, December 2025](https://inertiallabs.com/wp-content/uploads/2026/02/VINS_Integrated_with_Vantor_Raptor_Guide_Technical_Report_DEC2025.pdf).
- **The principle in a patent.** A [patent](https://patents.google.com/patent/US9360321B2/en) of the company's Swedish predecessor describes rendering a picture from the 3D database at an assumed pose, comparing it with the camera picture by normalised cross-correlation, and repeating. That is the comparison our brightness matching uses. No source says that the product works exactly this way.

### VNS01 (UAV Navigation)

Source for this section: the company's [architecture paper](https://www.uavnavigation.com/company/blog/gnss-denied-architecture-unmanned-aerial-systems), September 2026.

- **Visual odometry.** Follows distinctive points between consecutive frames and corrects the speed. About 1 percent drift. It works "as long as there is enough luminosity".
- **Template matching.** Recognises features that were stored with coordinates on earlier flights with GNSS. Valid from half to twice the height of the original flight.
- **Terrain navigation.** Compares the profile from a radar altimeter with an elevation model, at low altitude. Reported error: 239 ± 16 m over 187 km.
- **Map matching.** A deep-learning model aligns the camera image with satellite imagery, at higher altitude. Reported error: 31 ± 9 m over 36 km. It follows the public [STHN paper](https://arxiv.org/abs/2405.20470).
- **Fusion.** The fixes go into the autopilot's own estimator as corrections at a low rate.
- **Hardware.** The unit weighs 100 g and uses a global-shutter camera.
- **Stated limits.** Not available above clouds or over the ocean. Oceans and deserts are named as a challenge.

### Night

- **Neither product claims night operation with an ordinary camera.**
- **Raptor at night means an infrared camera.** The one published night flight used a FLIR Boson+ camera over about 58 km. The report's table gives an error of 5.0 m and its text says under 15 m. Source: [Inertial Labs white paper](https://inertiallabs.com/gnss-denied-performance-of-vision-based-positioning-apnt-solution/).
- **How the infrared picture is matched is not explained.** The white paper only says that patterns from a "day or IR camera" are compared with maps derived from satellite imagery. The developer pages we saved do not contain the words night, infrared or thermal. A [2019 patent](https://patents.google.com/patent/US11164338B2/en) of the company describes map textures for "low ambient light conditions"; whether the product uses them is not stated anywhere.
- **VNS01 makes no night claim.** Without light its visual odometry is unavailable. What remains is terrain navigation with the radar altimeter, which needs no camera, and dead reckoning. Its map-matching model was pretrained on a dataset of thermal and daylight images, which points to a thermal option, but no source states a thermal camera.

### What this means for us

- **Our chain on ALTO has the same parts.** A prior position, a search around it that grows with the uncertainty, a match score, a threshold below which a fix is rejected, and fusion with dead reckoning.
- **Our accuracy is in the published range.** Our median of 26 to 31 m compares with the 31 ± 9 m that VNS01 reports for map matching. Our conditions are easier: a short section, and reference images along the route only.
- **Night needs another sensor.** Our result is for daylight. The known route to night is an infrared camera against the same reference data.
- **Water is excluded by both vendors.** It is an open problem for everyone, and it is first on our list for Taiwan.
- **Quote vendor figures with their document.** They vary between the vendor's own papers: accuracy "under 10 m" and "under 7 m", minimum altitude 50, 100 and 120 m.

## Eagle Eyes and the systems fielded in Ukraine

Researched on Saturday at 13:00, after Dustin noticed that Eagle Eyes looks like our solution. We found no technical paper for any of these systems. What follows comes from press reports and makers' pages; the Ukrainian wording was checked where the English reports summarise it.

### What Eagle Eyes is

- **Software of Ukraine's special operations forces,** reported by The Economist on 29 May 2024: "Using artificial-intelligence (AI) algorithms, the software compares live video of the terrain below with an on-board map stitched together from photographs and video previously collected by reconnaissance aircraft." Sources: [UAS Vision](https://www.uasvision.com/2024/06/04/ukraines-special-forces-have-drones-that-fly-without-gps/), [Business Insider](https://malaysia.news.yahoo.com/ukraines-special-forces-developed-tech-122748126.html), [online.ua](https://news.online.ua/en/ukraines-army-creates-software-to-protect-its-drones-from-russian-jamming-879390/), and a [Ukrainian summary of the Economist article](https://foreignukraines.com/2024/06/10/ukrainian-it-companies-have-developed-tools-to-protect-drones-from-russian-radio-electronic-interference/).
- **It also recognises targets** (tanks, armoured vehicles, rocket launchers) and can drop a bomb or dive onto them without the operator. The unit names Russian jamming stations as its first target.
- **Scale.** In spring 2023 three special-forces teams used it. By mid-2024 it was "cheap enough to be used on kamikaze drones" and in wide use, also on fixed-wing drones with a range of about 60 km. Ukraine had several hundred drones with optical navigation in autumn 2023 and close to 10,000 in mid-2024.
- **Not public:** its accuracy, its hardware, and how the matching works.

### The same idea as ours, and older than both

A downward camera, a map on board, the live picture matched against the map, and nothing that can be jammed. The Tomahawk cruise missile's camera fix (DSMAC) has worked this way since the 1980s ([Irani and Christ 1994](https://secwww.jhuapl.edu/techdigest/Content/techdigest/pdf/V15-N03/15-03-Irani.pdf), in [reading-notes.md](reading-notes.md)). The jury will know this. What we can claim is how well a weekend version is measured and how honestly it reports.

### Where Eagle Eyes differs from ours

| | Eagle Eyes | Ours |
|---|---|---|
| Map | Stitched from recent reconnaissance flights over the same ground | Satellite or aerial photos, 2 to 5 years old |
| Matching | Described as AI | Brightness correlation, the same family as Tomahawk's; one CPU core |
| Targets | Finds and attacks them | Out of scope |
| Evidence | In combat use since 2023; no numbers published | Recorded flights in the USA and China, one held-out test, numbers in [findings.md](findings.md) |

The map is the difference that matters. Our worst result, UAV-VisLoc flight 01, fails because the ground was rebuilt between the photos and the map. A map from a recent reconnaissance flight avoids that. Tomahawk's planners worked by the same rule: a map has a limited life and is compared with new imagery at intervals, and a map made from summer imagery "is unlikely to correlate reliably during winter".

### Numbers from the same report, as a yardstick

- **MindCraft.ai (Lviv):** "For small drones with inexpensive optical navigation, the ideal flight altitude is about 500 metres", and the system "must lock onto at least one landmark per minute to avoid deviating from course by more than 50 metres". Our UAV-VisLoc flights are at 400 to 550 m; our fixes come every 300 m, about every 22 s there; our alert limit is 50 m.
- **Where it works best:** "near intersections, power lines, isolated trees, large buildings and adjacent water bodies".
- **Fielded systems that publish a number claim 15 to 20 m.** Osiris: 15 m CEP by day on the [maker's page](https://www.delian.ai/osiris); under 20 m on average over about 3,000 km of trial flights in Ukraine at 70 to 2,000 m ([Defence Express](https://en.defence-ua.com/weapon_and_tech/details_revealed_on_how_ukraine_tested_a_dsmac_style_navigation_system_on_mid_strike_drones-18750.html)). OSCAR: about 20 m. Our median is 28 to 31 m on the development flights and 60 m on held-out flight 04. We do not claim to match them.

### What we take from it

1. **The pitch.** The principle is proven in combat. What we add in a weekend is an open version, tested on flights it never saw, that says how sure it is and where it breaks.
2. **Map age is the lever, and the simulator can show it.** Run the navigator twice on the same simulated flight: with the 2020 image as the map (the ground itself, a fresh map as Eagle Eyes has) and with the 2018 image (two years old). The difference is the price of an old map. It costs minutes once the simulated flight exists.
3. **A forecast before the flight of where fixes will hold.** Tomahawk's mission planning "predicts the likelihood that the true peak will be larger than all the sidelobes" for every scene. Ours would be: at points along the planned route, how unique the map is inside the search circle. It is the navigability map of section 3 above, for camera fixes. One to two hours; after the simulator.
4. **Not worth doing:** Tomahawk's later rule of adding up the match scores of several frames (Block IIA). It needs frames that see different ground. Ours overlap and agree on the same wrong place (findings 3.6); frames far enough apart are what the confirmation of large jumps already compares.
5. **If the jury asks why we use no AI:** correlation is cheap and its failures can be measured. A learned matcher helps with seasons and light (the season-invariant paper on our reading list). It cannot help where the ground was rebuilt, as on flight 01.

## Correction to an earlier statement

After the first experiments I wrote that I had found nothing like the learned-wind result. That was wrong. Estimating wind and using it for dead reckoning is standard in the PX4 and ArduPilot autopilots. What remains ours is learning the wind from camera and terrain fixes when GNSS is never available, and measuring what that buys over water.
