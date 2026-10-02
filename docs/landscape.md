# Landscape: existing products, their limits, and what we should build

Research on Friday evening, after the first experiments in [experiments.md](experiments.md). It answers three questions: what exists for navigation without GNSS, where it falls short, and what a team of six can add in a weekend.

## Short version

- The high end is solved. Laser velocity sensors, star trackers and magnetic-anomaly navigation hold position over land and sea, on aircraft that can carry and afford them.
- Taiwan is already adopting a foreign product. Maxar's Raptor is being rolled out to Taiwan's drone industry through AIDC. It needs Maxar's own 3D terrain data.
- Open-source autopilots already estimate the wind and use it for dead reckoning when GNSS is lost. Our wind result is therefore known practice, with one catch: they learn the wind while GNSS still works.
- The gaps we can address: cheap sensors with open maps, a system that knows when it is wrong, and a way to tell before the mission where navigation will hold.
- Proposal: build the navigator with open data and cheap sensors, add an integrity check, and add a map of Taiwan that shows the expected position error along a planned route.

## Existing products

| Product | How it works | Reported performance | Limits for our case |
|---|---|---|---|
| [Maxar Raptor](https://vantor.com/product/mission-solutions/raptor/) | Ordinary camera matched against Maxar's Precision3D terrain data | Under 10 m RMSE, works at night and at low altitude | Needs Maxar's proprietary data. Nothing to match over open water |
| [UAV Navigation VNS01](https://www.gpsworld.com/uav-navigation-enhances-vns01-for-precision-drone-navigation-in-gnss-denied-environments/) | Visual navigation plus terrain-referenced navigation and satellite map matching | Not stated in the sources I read | A dedicated hardware unit |
| [OSCAR](https://thedefensepost.com/2026/01/29/ukraine-drones-vision-navigation/) (Ukraine) | Camera matched against mapped landmarks | About 20 m, in fog and at night | Land only |
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

This proposal became the plan in [PLAN.md](PLAN.md), which adds roles, timeline and demo.

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

## Correction to an earlier statement

After the first experiments I wrote that I had found nothing like the learned-wind result. That was wrong. Estimating wind and using it for dead reckoning is standard in the PX4 and ArduPilot autopilots. What remains ours is learning the wind from camera and terrain fixes when GNSS is never available, and measuring what that buys over water.
