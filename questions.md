# Questions for tomorrow

Questions only the team can resolve. Each time, the night continued with the stated assumption.

## Orchestration

1. ~~**ALTO Val.zip**~~ — resolved overnight: Track C retrieved it through the new Dropbox link (`data/raw/alto/Val.zip`, 1.86 GB), along with `UAV_Round2_Train.zip` (11.3 GB). Nothing to do unless Dustin wants to compare checksums with his copy.
2. **Target hardware**: for the budget of about 500 dollars, do you already have a drone or stereo camera model in mind? The altitude band covered by stereo depends directly on the distance between the two cameras.
   Assumption retained: compare several commercial stereo cameras without choosing one.
3. **Demo flight altitude**: are we aiming for 30 to 50 m, or 100 to 150 m above ground?
   Assumption retained: test both bands separately.
4. **NLSC imagery rights**: can someone confirm with the organizers or NLSC whether the tiles can be stored offline for the demo?
   Assumption retained: use only a few samples for research; document the rights rather than assume them.
5. **Organizers' dataset** (IMU, speed, heading, reference position of a moving platform): the link is on the participants-only page. It is not public and has not been retrieved. Who can place it in `data/raw/organiser/`?
   Assumption retained: nothing has been done with it; this is probably the dataset the jury expects for the baseline.

## Track B: altitude, speed, and heading

1. **Date and time of the demo or filmed flight**: the solar sensor is useless when the sun is almost at its zenith (May to August around midday in Taiwan) and less useful under cloudy skies. Which month and time?
   Assumption retained: October, between 9 and 16, with variable cloud cover.
2. **Attitude reference**: does the drone have a usable magnetometer, and what roll/pitch error does the autopilot report in flight? The solar sensor's heading error is about tan(elevation) × tilt error.
   Assumption retained: standard PX4/ArduPilot autopilot, 1 to 2 degrees of tilt error in flight.
3. **Terrain below the trajectory**: will the demo flight be over flat ground (Taichung, Wufeng highway) or hilly terrain? The barometer tracks ground height only if the terrain is flat or an onboard elevation model is allowed.
   Assumption retained: flat ground; Copernicus GLO-30 onboard elevation model allowed.

## Track C: data, replay, and simulation
- YouTube videos linked to PX4 logs (EasyStar FPV, `data/raw/px4_video/`): license not established. Can we show these images in the jury video, or use them internally only? Assumption retained: internal use only.
- Zurich AGZ: images described as “academic research without any limitations.” Does a hackathon jury video count as academic research? Assumption retained: yes, with attribution, to be confirmed.
- The simulator: do you want the author to apply the fix `data/processed/t_sim_rec/sim_patch.diff` (GNSS cutoff, recorder, stereo, orientation leakage) on the `simulations` branch? Nothing has been changed on that branch.
