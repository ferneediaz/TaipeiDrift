# Phone sun compass test

Our mentor's idea: use an iPhone to measure, for real, how accurately a camera can tell which way it points from the position of the sun. The number goes straight into our navigator and into the simulator, in place of a figure we took from a paper.

| When | Who | Time needed | Hand over |
|---|---|---|---|
| Today (Saturday) until 14:30, or tomorrow 09:00 to 09:45 | Anyone with an iPhone | About 40 minutes | Photos by AirDrop to Dustin |

Written Saturday 3 October, 14:00. Code: `baseline/scripts/phone_sun_compass.py`.

## Why we need this number

Our navigator adds up the drone's motion, seen by its downward camera, in the direction the drone is heading. If the heading is wrong, every step is turned sideways. With a heading 4° off, the drone ends up 1,000 × sin 4° ≈ 70 m to the side after 1 km. With 0.6° off, it is about 10 m.

Every drone has a compass, but it can be several degrees off (our models assume 4°), and motors and metal disturb it. The sun gives a direction that nothing on board can disturb: from where you are and what time it is, a standard formula says exactly where the sun stands. A camera that sees the sun therefore knows which way it points.

So far our "sun sensor" uses the accuracy of a purpose-built sensor from a paper (Fan et al. 2016, 0.1°). This test measures what an ordinary camera achieves. It also answers an open question from Alessandro: his filter can already use the sun, but he found that it needs "a sky-pointing camera, or a detector validated against a known sun direction" (`vio/README.md`). The phone test gives both.

## What the phone stands in for

| On the drone | In this test |
|---|---|
| A sun sensor, or a camera looking up | The iPhone's back camera, pointing straight up |
| Position and clock, known from GNSS before the jamming | The location and time stored in every iPhone photo |
| Level attitude, known from the IMU | The phone lying flat on a level table |
| A known turn of the drone | The phone turned by exactly 90° against a fixed right angle |

## The idea, from zero

**Where the sun stands.** From place and time, the formula gives two angles. The azimuth is the sun's compass direction, counted clockwise from north. The elevation is its height above the horizon. At NTU today at 14:00 the sun stands at azimuth 234° (southwest) and elevation 46°.

**Where the phone points.** In the photo the sun is a bright spot at some pixel. The camera calibration (15 to 20 photos of a chessboard, processed with OpenCV) tells us how pixels translate into angles. From the spot's position we get the sun's direction relative to the phone's top edge. If the sun lies 30° clockwise from where the top edge points, the top edge points to 234° − 30° = 204°.

**How we know the error without a true north.** We have no exact north on the table, but we know the turns. Turning the phone by exactly 90° must change the measured heading by exactly 90°. Eight photos at 0°, 90°, 180° and 270°, twice, give eight readings; how far they scatter around their 90° steps is the error of one reading:

```
readings after removing the 90° steps:   +0.4   −0.3   +0.9   −0.6   +0.1   −0.8   +0.5   −0.2
average                                      0.0
spread (standard deviation)                  0.6°   <- the error of one heading reading
```

These numbers only show the arithmetic; the real ones come from the photos. One more check comes free: the height of the sun measured in the photo must match the formula's elevation. If it does not, the table is not level or the calibration is off.

## When: the sun has to be in the picture

Lying flat, the phone sees only the sky near straight up. The normal 1x camera reaches about 35° from straight up, the 0.5x ultra-wide about 50°. The sun is in the picture only when it stands high enough, so **we use the 0.5x camera**.

| Time at NTU | Sun direction | Height | With 0.5x |
|---|---|---|---|
| Today 13:30 | 226° southwest | 51° | in view |
| Today 14:00 | 234° southwest | 46° | in view |
| Today 14:30 | 240° west-southwest | 40° | near the edge |
| Today 15:00 | 246° west-southwest | 34° | too low |
| Tomorrow 08:30 | 115° east-southeast | 35° | too low |
| Tomorrow 09:00 | 121° east-southeast | 41° | near the edge |
| Tomorrow 09:30 | 128° southeast | 46° | in view |

Tomorrow's window ends with the code freeze at 10:00, so start by 09:15. Choose a spot with open sky towards the sun's direction and no cloud over the sun. The picture reaches furthest in its corners: if the sun is missing at one of the four turns, rotate the whole set-up so that the sun appears near a corner of the picture.

## Steps

### A. Set up the iPhone (5 min, indoors)

1. Settings → Privacy & Security → Location Services → Camera: *While Using the App*, with Precise Location on. Every photo then carries its place and time.
2. Settings → Camera → Formats: *Most Compatible* saves JPEG. HEIC also works; our script converts it on the Mac.
3. Settings → Camera: leave *Lens Correction* on (it is on by default). It keeps the 0.5x picture free of fisheye bending.
4. In the Camera app choose **0.5x**. Use 0.5x for every photo in this test, the chessboard ones included: the calibration only holds for the lens it was made with.

### B. Calibrate the camera with a chessboard (10 min, indoors)

1. Show the chessboard image full screen on a laptop with the brightness up. Dustin has it as `chessboard_9x6_inner_corners.png`; any chessboard of 10 × 7 squares (9 × 6 inner corners) works.
2. Take 15 to 20 photos of it at 0.5x: from different distances, tilted up to about 30°, and with the board placed in different parts of the picture, several of them near the corners. The sun will often appear near the edge of the picture, so the edges need to be calibrated well.
3. Every photo must show the whole board, sharp. Hold still; a blurred photo is ignored.

### C. Photograph the sun (15 min, outdoors)

1. Find a table or bench in direct sun. Check that it is level with a spirit level or a level app; it should read 0°.
2. Tape a book or box with a square corner to the table. This corner fixes the four turns of exactly 90°.
3. Open the Camera at 0.5x, point it at the bright sky, press and hold until *AE/AF LOCK* appears, then drag the exposure down as far as it goes. The sun should become a small white disc in a dark sky.
4. Lay the phone screen down, back camera up, pushed into the corner. Take the photo with a volume button.
5. Pick the phone up and check that the sun is in the photo. If not, see the note on corners above.
6. Turn the phone a quarter turn, push it into the corner again at the same spot, and take the next photo. Do 0°, 90°, 180°, 270°, then the same four again: eight photos in all.
7. Keep your shadow and your hands out of the sky above the phone while it shoots.

### D. Hand over (5 min)

1. Send all photos to Dustin's Mac by AirDrop. In the share sheet tap *Options* and check that *Location* is on, so the place and time stay in the files.
2. Do not send them through WhatsApp, LINE or Messenger: these apps remove the place and time.
3. Say which photos are chessboard and which are sun, or put them in two albums.

### E. Run the analysis (5 min, on the Mac)

1. Chessboard photos into `data/raw/phone_sun/chessboard/`, sun photos into `data/raw/phone_sun/sun/`.
2. From the repository root: `python baseline/scripts/phone_sun_compass.py`
3. It calibrates the camera, finds the sun in every photo, computes the sun's position for each photo's place and time, and prints the heading per photo, the error of one heading reading, and the elevation check. Files go to `outputs/phone_sun/`.

## What comes out, and what we decide

| Measured error of one reading | What it means | What we do |
|---|---|---|
| About 1° or better | A camera works as well as our sun-sensor model assumes at these sun heights | Keep the sun-sensor results; the slides say "measured with an iPhone" |
| 1° to 3° | Still better than the 4° compass offset we assume | Put the measured number into the model and rerun |
| Worse, or the sun is often not found | An ordinary camera is not good enough this way | Report it, keep the compass as the baseline |

For comparison, our current model predicts 0.7° to 1.0° for a drone at a sun height of 35° to 46°, most of it from a 1° tilt error of the drone. A phone on a level table has less tilt, so it measures mainly the camera's own share.

## How it feeds into our navigator

The heading enters our algorithm in three places, and the measured number replaces the assumed one in each.

| Where | What the heading does there |
|---|---|
| Map fixes | Before each position fix, the camera picture is turned north up with the heading, so that it can be laid onto the north-up aerial map. The matcher tries a few degrees either side; a larger heading error makes fixes fail. |
| Between fixes | Each camera step is turned into north and east with the heading. A wrong heading turns every step sideways: 70 m after 1 km at 4°, 10 m at 0.6°. This decides how far the drone drifts before the next fix and how wide the next search must be. |
| Fix offset | Where the camera looks compared with where the drone is (it looks slightly backwards when the drone tilts forward) is turned with the heading too. |

Then we rerun the same tests with the measured sun sensor:

- **The heading model.** `baseline/src/sensors/heading.py` computes a sun sensor's heading error from two numbers: the sensor's own error and the drone's tilt error. The phone gives the first one for an ordinary camera; it goes into the configs in place of 0.1°.
- **UAV-VisLoc, real drone photos from China.** The sun-sensor runs on the development flight 03 and the held-out flights 01 and 04. With the paper's number, the sun sensor already cut flight 01's median error from 306 to 177 m. The phone tells us whether that holds with a camera we can actually buy.
- **The simulated flight over Wufeng.** Our Gazebo flight (4.8 km at 100 m, the 2020 aerial image as the ground, the 2018 image as the map) uses a compass model today. With the measured number it gets a sun sensor, simulated from the drone's true attitude plus the measured error. This is the "simulate how the drone would do" part of the mentor's idea.
- **Alessandro's filter.** His filter can take the sun direction as an attitude measurement: together with gravity it fixes the full attitude, heading included. The phone photos are the check of his detector against a known sun direction that his notes ask for, and he can run his detector on the same eight photos.

## What this test does not show

- A phone on a table is still. A drone shakes and tilts, and its tilt estimate from the IMU (about 1°) adds heading error that grows with the sun's height. Our model includes that part; the phone measures the camera's part.
- It needs the sun: no cloud over it, daytime only. When the sun stands nearly overhead (Taiwan around noon from May to July) it says almost nothing about the heading.
- One phone on one afternoon is a first measurement. It is enough for the slides; a full validation needs more phones and more days.

## Who does what

| Who | What | When |
|---|---|---|
| Teammate with an iPhone | Steps A to D: set-up, chessboard photos, eight sun photos, AirDrop | Today by 14:30, or tomorrow 09:00 to 09:45 |
| Dustin | Step E: photos into the folders, run the script | Right after |
| Claude | Measured number into the heading model; rerun UAV-VisLoc and the simulated flight; working doc and a chart for the slides | Within the hour after |
| Alessandro (optional) | His sun detector on the same photos, as a cross-check | Any time |

Sun positions computed for NTU (25.017° N, 121.540° E) with the NOAA formulas our code uses (`baseline/src/sensors/heading.py`).
