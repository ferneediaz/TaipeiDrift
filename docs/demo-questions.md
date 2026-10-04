# Questions the demo video may raise

For whoever stands in front of the jury. Each answer is short enough to say aloud, and every number comes from
seventeen logged flights of the demo route (table and method: `docs/simulation-results.md`, section "The demo flight
of the video, tested on twelve flights"). State: Sunday 4 October, 02:50.

## If there are only twenty seconds

1. It is a simulation, and the autopilot flies on the true position. Our estimate runs alongside and is scored
   against the truth.
2. In six flights with the video's settings the estimate was 4 to 10 m off without GNSS in five, and 24 m in one.
   The inertial sensors alone ended 150 to 520 m away in those flights.
3. Over open water the camera has nothing to track. There the ships' radio holds the position to some tens of
   metres and keeps the error from growing.

## The likely questions

**1. Is this real or a simulation?**
A simulation (Gazebo). The drone, its sensors with their noise, the ships and their radio are simulated. Our
real-world evidence is separate: a camera on a cart that tracks its own path over a floor, and the replay on real
drone photos that belongs to the map navigator on the slides.

**2. Does the drone steer by your estimate?**
No. The autopilot steers by the simulator's true position; our estimate runs alongside and is compared with the
truth. The last caption of the video says so. Flying on the estimate is the next step.

**3. One flight can be luck. How often did you fly it?**
Six times after the settings were fixed, with new random sensor noise each time. Five flights were 4 to 10 m off
in the median, one was 24 m. The video shows a typical flight (8 m), not the best one (4 m).
If they ask about the 24 m: the heading was 4 degrees off when GNSS went. One degree costs 8 m over this route.

**4. What is the red line?**
The same flight with only the inertial sensors and the barometer, which every drone has. After about 55 s it is
150 to 950 m away, depending on that flight's sensor noise; in the video 241 m. After five minutes it is 3.5 km.

**5. Why does your error jump to 50 or 100 m over the water?**
Over open water the camera has nothing to track. There the estimate rests on the inertial sensors and the ships'
radio, and a radio fix is rough, 30 to 65 m. Late in the crossing the error climbs for a few seconds, to between
45 and 100 m in our flights, and comes back down when the camera sees the island again.

**6. Then why the radio, if one fix is 30 to 65 m rough?**
Because its error does not grow with time. The camera's speed is precise, but its small errors add up. In a flight
of almost five minutes, back and forth over the strait, our estimate stayed between 6 and 22 m; the same filter
without the radio drifted to 95 m in the median and 340 m at worst. In six of the seven crossing flights at the
normal pace our estimate over water was better than the camera alone and better than the ships alone.

**7. The dashboard shows "EXCEEDED" in orange. What is that?**
The filter also says how sure it is, and that statement is too optimistic. Over the flights the true error was
inside its stated bound anywhere between never and always. It treats the camera's and the radio's errors as random
from one reading to the next, and they are not. The map navigator on the slides does hold its bound on the sealed
flights; bringing that rule into the live filter is open work.

**8. What if the ships switch AIS off, or there are none, or one reports a false position?**
Without a transmitter there is nothing to take bearings on, and over water the drone drifts like the red line until
it sees land. AIS is our example of a signal that is already there: the method needs a transmitter whose position
is known, such as a coastal radio station, a radar site or a friendly vessel. A ship that reports a false position
is not handled yet.

**9. Can the radio be jammed like GNSS?**
A GNSS jammer does not touch it: AIS is a different band (VHF, 162 MHz), and a ship a few kilometres away is far
stronger at the drone than a satellite in orbit. A jammer built for that band would work. Then the camera and the
inertial sensors carry on alone, as in the camera-only row.

**10. Night, fog?**
The radio and the range finder need no light. The camera does; at night it would need a thermal camera. Not
tested.

**11. The sea in the video looks like a still picture. Real waves move.**
Right. In the simulation the deep sea has almost no texture, so the camera gets nothing there, as over real water.
Near the shore the seabed shows through and the simulated camera tracks it; real waves and glare would make that
harder. We then flew the hard case seven times, a sea without any texture, and the new video uses it. With the
ships' radio running before the drone leaves the coast the estimate over water is about 10 m off in the median,
with moments of 45 to 100 m; with the radio starting only at the coast it is 16 to 31 m, with moments up to 120 m.

**12. How long can it go without GNSS?**
The video shows 53 s and 470 m. The longest flight was 289 s, about 2 km: 11 m off in the median, 44 m at worst,
and the error did not keep growing.

**13. Were the settings tuned on this very flight?**
They were chosen on Saturday evening on five test flights on this route. After that the filter's settings stayed
as they were, and we flew eleven more: the two takes for the video, four repeats, wind, an earlier loss of GNSS,
two with a sea without texture and the five-minute flight. All of them in the same world with the same three ships. Another
coast or other ship positions are not tested.

**14. How does the video fit the numbers on the slides?**
They are two parts of one system. The numbers on the slides are the map navigator: the camera matched against a
stored aerial image, over land. The video shows what the drone does where there is nothing to match, over water:
speed from the camera and bearings on ships. (Check this answer against the final deck.)

**15. How does it know its heading?**
From the gyroscope, corrected by the GNSS course while GNSS is there, and by the ships' bearings afterwards. It
has no compass, and in the simulation it starts from the true heading. In the flights the heading was 0 to 4
degrees off when GNSS went, and that is the main reason the results differ from flight to flight. The map
navigator on the slides uses a sun sensor for this.

**16. Does the camera part work over a city or a forest?**
It needs ground that is roughly flat under the picture. Over flat ground it measured the speed to 0.16 m/s. Over
buildings it failed in our city test (4.5 m/s off), because one range reading cannot describe roofs and streets at
once.

**17. Wind?**
One flight with 6 m/s wind and gusts: 5 m off in the median, 23 m at worst. One flight only.

**18. What does it need on board?**
A downward camera, a single-beam range finder, the IMU and barometer the drone already has, and an AIS receiver
with a small direction-finding antenna. The software of the demo runs on a laptop processor without a graphics
card. For weight and cost see Felix's report (those numbers are not checked here).

**19. Is the scoring right?**
Each estimate is compared with the simulator's true position at the same instant. Dan's own scoring script,
written independently, gives the same numbers on the same flight: 9.4 m against 9.6 m in the median, 59.7 m
against 60.5 m at worst.

**20. Can you run it live?**
Better not. The simulator's window crashed at the start in 2 of about 18 starts that night, and a flight takes
four minutes. The video is a recording of one such run, with every number read from its log.

## Words to avoid

- "8 m accuracy". Say: 4 to 10 m in five of six flights of this simulated scenario, 24 m in the sixth.
- "It knows when it is wrong", for the live filter. That holds for the map navigator on the sealed flights only.
- "It works over open sea". Say: with ships in range it holds some tens of metres over water.
- "It cannot be jammed". Say: a GNSS jammer does not reach it.
- "Real flight", for anything in this video.

## The flights behind the numbers

Median and worst error of our estimate without GNSS, in metres; "inertial" is the inertial sensors alone at the
arrival.

| Flight | Seconds without GNSS | Median | Worst | Camera only, median | Inertial |
|---|---|---|---|---|---|
| The flight the settings were chosen on | 58 | 8.1 | 36 | 8.1 | 953 |
| Take 1 | 53 | 8.5 | 66 | 7.0 | 405 |
| Take 2, the video | 53 | 7.8 | 56 | 9.0 | 241 |
| Repeat 1 | 57 | 8.7 | 60 | 18.7 | 518 |
| Repeat 2 | 59 | 3.9 | 11 | 28.7 | 153 |
| Repeat 3 | 58 | 9.7 | 43 | 31.1 | 438 |
| Repeat 4 | 59 | 24.3 | 64 | 33.7 | 350 |
| Wind 6 m/s, gusts | 62 | 5.0 | 23 | 54.4 | 379 |
| GNSS lost at 11 s | 75 | 8.2 | 34 | 39.6 | 308 |
| Sea without texture, matte | 57 | 28.4 | 69 | 37.0 | 201 |
| Sea without texture, glossy (Alessandro's sea) | 58 | 22.2 | 119 | 27.5 | 507 |
| To the island and back | 289 | 10.6 | 44 | 94.9 | 3,477 |
