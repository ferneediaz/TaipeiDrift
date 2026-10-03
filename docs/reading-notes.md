# Reading notes

The papers behind our one thing, the camera as a position sensor once GNSS is jammed, and behind its next steps. Read on Friday and Saturday night, 2 and 3 October 2026. For each paper: what it did, how much of it was read, and what we take from it. The measurements of our own are in [findings.md](findings.md).

## What the reading changes for us

- **Our matcher is the right simple tool.** Correlation of brightness patterns came second of 12 classical scores in Kinnari 2021, almost tied with the best. In Kinnari 2022 it was the best of seven methods on textured ground with a map from a similar season (10.9 m, ahead of their neural network), and it failed under snow.
- **Keypoints fail between camera and map** in every paper that tried them: under 7 percent success in Patel 2020, 99.6 percent failure on thermal images in STHN, dropped in WildNav. On ALTO 2 of 100 were accepted, both wrong (findings, section 3.2).
- **A stronger check rests on agreement.** Our score threshold of 0.33 was tuned on the same section it is reported on. Tomahawk accepted a fix only if consecutive frames agreed with the inertial motion; UASTHN rejects a fix if crops of the same frame disagree. Neither needs a tuned number. Tested on Saturday for consecutive frames: it does not work for us (findings, section 3.6). Tomahawk compared against an accurate inertial system after GPS or terrain matching had narrowed the search; our frames 14 m apart see the same ground and agree on the same wrong place.
- **Heading is the weak spot in every system:** 2 km to converge without it (Kinnari), drift mainly from the compass (Honegger), compass errors of 15 degrees (LSVL, WildNav). This backs the sun sensor as a next step.
- **Cost is where we differ.** LSVL needs a Jetson Nano and 3.15 s per update, Raptor a GPU. Tomahawk's camera fix matched small black-and-white images with 1980s electronics, which fits our finding that 125-pixel images match as well as 500-pixel ones. Its setting differed from ours: GPS or terrain matching narrowed the position first.
- **What breaks it, according to the papers:** forest, fields and water (no distinct pattern), long shadows, season, night, outdated maps. Each of these needs one next step on the limits slide.

## Camera against a map

| Paper | Read | What it did | What we take from it |
|---|---|---|---|
| Kinnari, Verdoja, Kyrki 2021, [arXiv 2103.14381](https://arxiv.org/abs/2103.14381) (PDF in `research/`) | In full | Particle filter over position, heading and scale; camera tilted 50 to 60 degrees and projected to a top view; 12 matching scores compared; three real flights of 4 to 7 km at 92 m | Known start: 18 to 51 m error against 52 to 253 m for odometry alone. Unknown heading: about 2 km to converge. Turns the match score into a probability using the score distributions of right and wrong places |
| Kinnari, Verdoja, Kyrki 2022, [arXiv 2110.01967](https://arxiv.org/abs/2110.01967) (PDF in `research/`) | In full | A neural network that compares a camera patch with a map patch, trained for winter against summer | Real flights: 26.5 to 30.6 m after 2 km. Fails over dense forest, falls back to odometry over a lake. Assumes 2 m of odometry error per 100 m |
| Kinnari et al. 2023, LSVL, [arXiv 2212.03581](https://arxiv.org/abs/2212.03581) | In full | Finds a drone anywhere in 100 km², with a grid over all positions and headings | 12.6 to 18.7 m after 23 to 44 updates. On a Jetson Nano: 3.15 s per update, 461 MB of map for 6 km². Uses the spread of its estimate to say it is lost |
| Irani, Christ 1994, Tomahawk scene matching, [APL Technical Digest](https://secwww.jhuapl.edu/techdigest/Content/techdigest/pdf/V15-N03/15-03-Irani.pdf) | In full | The Tomahawk cruise missile's camera fix (DSMAC), one of three position sources next to terrain matching (TERCOM) and GPS | Not a GNSS-denied design: DSMAC updates "follow TERCOM or GPS updates", so it searched an area that was already small, and its frame check compared against an accurate missile-grade inertial system. What carries over is the image side: binary, low-resolution images, since fine detail rarely helps. A fix only if 2 of 3 frames agree with the inertial motion. Summer maps fail in winter; moving shadows shift the match; a strobe at night |
| Shan et al. 2015, [arXiv 1703.10125](https://arxiv.org/abs/1703.10125) | In full | Optical flow predicts, gradient patterns match a Google map, a bad match is dropped | Closest to our design. 6.8 m against 169 m for optical flow alone, but on a 300 by 150 m area for 3 minutes |
| Patel, Barfoot, Schoellig 2020, ICRA | In full | Matching against images rendered from Google Earth's 3D model | Under 3 m at 40 m height from sunrise to sunset. Hard: vegetation and long shadows. Needs hundreds of MB of images for 1 km |
| Gurgu et al. 2022, WildNav, [arXiv 2210.09727](https://arxiv.org/abs/2210.09727) | In full | Learned keypoint matching against satellite tiles, 120 m height | 77 of 126 photos located (15.8 m mean error); the other 49 wrong and not flagged. Funded by the Finnish defence ministry |
| Bianchi, Barfoot 2021, [arXiv 2102.05692](https://arxiv.org/abs/2102.05692) | Method and setup | Compresses map images with a network trained for one route | Under 3 m and fast, but 20 hours of training per route and 42 reference images per metre |
| Honegger et al. 2013, PX4Flow, ICRA | In full | Optical flow sensor on a microcontroller | 250 Hz on a Cortex M4 with 64 by 64 pixels. Outdoor drift came mainly from the compass |

## The check

| Paper | Read | What we take from it |
|---|---|---|
| Zhu, Meurer, Günther 2022, Integrity of visual navigation, [NAVIGATION 69(2)](https://doi.org/10.33012/navi.518) | In full | The aviation vocabulary: the dangerous case is a large error while the system says all is well ("hazardously misleading information"). A protection level is compared with an alert limit. More checks do not always lower the risk |
| Xiao, Loianno 2025, UASTHN, [arXiv 2502.01035](https://arxiv.org/abs/2502.01035) | Problem and method | Rejects a fix if crops of one frame disagree on the position: 7 m with 3 percent of fixes rejected. Lists six causes of failure: no texture, damaged image, rotation errors, look-alike places, leaving the map, outdated map |

## Datasets

| Paper | Read | What we take from it |
|---|---|---|
| Cisneros et al. 2022, ALTO, [arXiv 2207.12317](https://arxiv.org/abs/2207.12317), and the competition readme | In full | Reference images are free US government aerial photos at 0.6 m. The sample has three parts of one 150 km flight: 24,701 images for training, 3,979 for validation. The full dataset adds an IMU and a laser altimeter, not in our sample |
| Dai et al. 2023, DenseUAV, [arXiv 2201.09201](https://arxiv.org/abs/2201.09201) | Skimmed | Image retrieval over 14 campuses in Zhejiang at 80 to 100 m. 17 percent of top matches wrong, none flagged; the position is that of a 20 m grid cell. Ilhan's review covers it |
| Xu et al. 2024, UAV-VisLoc, [arXiv 2405.11936](https://arxiv.org/abs/2405.11936) | In full | Real drone images over 11 places in China, 400 to 2,000 m high, with satellite maps taken years later. A candidate second test set after the hackathon |
| Fonder, Van Droogenbroeck 2019, Mid-Air, CVPR workshops | IMU and drone sections | Its documentation and its data disagree on the gyroscope axes (findings, section 2.2) |

## Next steps: heading from the sun, and night

| Paper | Read | What we take from it |
|---|---|---|
| Wei, Xing, Li, You 2011, Tsinghua, [Sensors 11(10)](https://doi.org/10.3390/s111009764) | In full | An N-shaped slit over a single row of pixels: sun direction to 0.08 degrees, 14 readings per second, 300 mW, 133 g |
| Fan, Peng, Gao 2016, Beihang, [Rev. Sci. Instrum. 87](https://doi.org/10.1063/1.4958696), the mentor's paper | Abstract only | A V-shaped slit and a microcontroller: 0.1 degrees, 25 readings per second, 200 mW. Behind a paywall with no open copy; the NTU library may have access |
| Xiao et al. 2024, STHN, [arXiv 2405.20470](https://arxiv.org/abs/2405.20470) | In full | Thermal camera at night against satellite images: 12 m within 512 m of the last known position, on a GPU |

Our own reasoning, not from a paper: the sun gives a heading only when it is away from the zenith. On the Tropic of Cancer it stands nearly overhead at midday in early summer, so a sun sensor in Taiwan gives little heading around noon from May to July.

## Not read

Conte and Doherty 2009 (aerial image matching with a Kalman filter), the 2025 review in Satellite Navigation, Couturier and Akhloufi 2021 (review of absolute visual localization), Dupeyroux 2019 (heading from sky polarisation) and the radio-positioning work around Kassas. They touch only the background and the next-steps slide.
