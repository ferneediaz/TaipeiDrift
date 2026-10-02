# How position fixes and the particle filter work

An explanation of the two ideas we take from the paper "Season-invariant GNSS-denied visual localization for UAVs" by Kinnari, Verdoja and Kyrki (2022): matching a camera view against a map, and Monte Carlo localization. Written for the whole team, with small numbers.

- Paper: [arXiv 2110.01967](https://arxiv.org/abs/2110.01967), PDF in [research/](../research/)
- The earlier paper by the same authors, which introduces the method: [arXiv 2103.14381](https://arxiv.org/abs/2103.14381), PDF in [research/](../research/)
- Where this fits: position fixes, the integrity check and the filter in [PLAN.md](PLAN.md)

Both ideas answer one question: if the drone were at this position, would the camera see what it sees now? Matching scores one guess. Monte Carlo localization manages a thousand guesses at once.

## Part 1: matching images

### Step 1. Make the camera image look like a map

The drone image is turned into a straight-down view at 1 m per pixel, the same scale as the map. With a camera that already points down and a known distance to the ground, this is a simple rescaling.

### Step 2. Cut out what the drone should see

Take one guess of where the drone is. A guess has four numbers:

- x, y: the position on the map
- φ (phi): the heading, which way the drone faces
- s: a scale factor, in case the height estimate is slightly off

From the map, cut the 96 m by 96 m square that the camera would show if this guess were right.

### Step 3. Compare the two squares

A function f takes the camera square and the map square and returns one number c between 0 and 1, the similarity score.

- In the 2021 paper, f is a classical formula that compares brightness patterns. They test twelve and find two that work best (Moravec and zero-normalised cross-correlation).
- In the 2022 paper, f is a neural network with two identical halves. Each half looks at one square, and a small final network compares what they found. It was trained on pairs of the same place in different seasons (target 1) and pairs of different places (target 0). That is how it learns to ignore snow and leaf colour.

### Step 4. Turn the score into a probability

A score of 0.9 does not mean "90 percent sure". So the authors measured, on test data, how scores are distributed:

- p(c | match): how often a correct position produces score c
- p(c | no match): how often a wrong position produces score c
- p(c | outlier): a flat value for "something unexpected", so the system never becomes fully certain

Then they apply Bayes' rule. It asks: of all the ways to get this score, what share comes from a correct position?

```
probability of match  =       p(c | match) × prior of match
                         ------------------------------------------
                         the same product, summed over all three cases
```

The priors are how likely each case is before looking: 0.475 for match, 0.475 for no match, 0.05 for outlier.

### With numbers

The distribution values below are made up to show the arithmetic. The paper reads them off its own histograms.

A high score, c = 0.9:

- correct positions give 0.9 often: p(c | match) = 4.0
- wrong positions rarely do: p(c | no match) = 0.2
- outlier, flat: 1.0

```
top:     4.0 × 0.475                          = 1.9
bottom:  1.9 + (0.2 × 0.475) + (1.0 × 0.05)   = 1.9 + 0.095 + 0.05 = 2.045
result:  1.9 ÷ 2.045                          = 0.93
```

A low score, c = 0.3, with p(c | match) = 0.3 and p(c | no match) = 2.5:

```
top:     0.3 × 0.475                          = 0.1425
bottom:  0.1425 + (2.5 × 0.475) + 0.05        = 0.1425 + 1.1875 + 0.05 = 1.38
result:  0.1425 ÷ 1.38                        = 0.10
```

This step is the core of our integrity check: it says how much a fix should be trusted.

## Part 2: Monte Carlo localization

"Monte Carlo" means using many random guesses in place of an exact formula. Each guess is called a particle. The paper uses 1000. The method is also called a particle filter.

### The loop

Repeated every 100 m of flight in the paper:

1. Move. Shift every particle by the movement the drone measured, plus a little random noise, because that measurement is imperfect.
2. Weigh. For each particle, run Part 1. Its weight is the probability that its predicted view matches the camera.
3. Resample. Draw 1000 new particles from the old ones. A particle with a high weight is copied many times. One with a low weight usually disappears.
4. Report. The estimate is the weighted average of all particles. How spread out they are is the uncertainty.

### With numbers

Four particles on a line, to keep it small.

| | Particle A | B | C | D |
|---|---|---|---|---|
| Position before | 10 m | 20 m | 30 m | 40 m |
| After moving +5 m | 15 | 25 | 35 | 45 |
| Weight from matching | 0.1 | 0.7 | 0.1 | 0.1 |

```
estimate = 0.1×15 + 0.7×25 + 0.1×35 + 0.1×45 = 1.5 + 17.5 + 3.5 + 4.5 = 27 m
```

After resampling you might have B, B, B, C: positions 25, 25, 25, 35. The guesses have gathered around 25 and the spread has shrunk. After a few rounds they collapse onto the true position. The paper calls that convergence.

### Why many guesses and not one

Several places can look alike: two road junctions, a row of similar fields. A method that keeps one best guess picks one and may never recover. With particles, some sit at each candidate place, and the wrong group dies out when the views stop matching.

### How it starts, and why the error first rises

At the start the particles are scattered over a 100 m by 100 m box with random headings, because the heading is unknown. Particles pointing the wrong way fly off in wrong directions, so the average gets worse at first. After about 2 km they have been eliminated and the error drops.

### Over a lake

Every particle sees the same featureless water, so all weights are roughly equal. Nothing is eliminated, the particles spread out with the movement noise, and the estimate relies on measured movement alone until land returns.

## What the paper reports

- Simulated flights over aerial images from different dates: 15 to 25 m error after convergence.
- Three real flights at about 92 m height: 26.5, 29.1 and 30.6 m after 2 km.
- Assumed error of the movement measurement: 2 m per 100 m flown and 1 degree in heading.
- Computing time: 0.33 s to cut the squares and 0.13 s to score them, for 1000 particles on a laptop.
- Weak spot: forest seen from a tilted camera, where trees look too different from the map.

## How the paper relates to our plan

| Plan part | What the paper gives |
|---|---|
| IMU baseline | Nothing |
| Camera speed | A benchmark: about 2 percent of distance flown |
| Position fixes | Its whole subject. Applies directly if we use ALTO, which has an aerial map |
| Integrity check | Step 4 above |
| Drift budget | The shape to expect: error rising without fixes, dropping when one arrives |

With Mid-Air alone there is no aerial map, so the method does not apply directly. The season problem is the same one that route memory faces. The authors' trained network is unlikely to work on Mid-Air, which is synthetic and at a much finer scale than the satellite images it was trained on.

## Which sections to read

About 20 minutes in the 2022 paper:

1. Abstract and Introduction
2. Section III-A, the filter
3. Section III-D, score to probability
4. Section IV-A, the experimental setting and noise values
5. Section IV-B, first four paragraphs, the simulation protocol
6. Figure 5, error against distance flown
7. Section IV-C, last paragraph, the lake
8. The paragraph on computing time before the Discussion

Sections III-B and III-C describe the neural network and its training, which we do not plan to rebuild.

## Terms

- MCL, Monte-Carlo localization: the particle filter described above.
- Orthophoto or orthoimage: an aerial image corrected to look straight down, like a map.
- VIO, visual-inertial odometry: estimating movement from camera and motion sensors. It drifts. Camera speed plays this role in our design.
- Yaw: heading.
- Convergence: the moment the particles collapse onto the right position.
