# Ground-referenced camera positioning from video (TestVideo)

Goal: show that a downward-looking camera at a **known constant height** can compute its exact
position over the ground at every instant, using only the texture of the surface below it.
That is the same principle as terrain-referenced navigation for a drone, scaled down to a cart
running at 64.1 cm.

| file | content |
|---|---|
| `A001_10031533_C001.mov` | flight video: iPhone (Blackmagic Cam), 2160×1214 @ 60 fps, 47.5 s, camera on a cart looking down |
| `A001_10031641_C002.mov` | calibration video: ChArUco board on an iPad, same camera settings |

## Run

```bash
./run_all.sh            # default height 0.641 m
./run_all.sh 0.6178     # e.g. with the tape-measure-derived height
```

The full chain takes about 30 min on this laptop. All results go to `output/`.

## What the program does

1. **`vidnav/calibrate.py` – camera calibration.** The board is detected automatically as
   7×5 squares, DICT_6X6, marker/square = 0.71; its physical size is not needed. ChArUco corners
   are found in every 2nd frame with sub-pixel refinement, and the sharpest frame per 10-frame
   window is kept. The lens model is chosen by 5-fold cross-validation, then bad views are
   rejected and the model is refitted.
   Result: fx = fy = 2689 px (±0.07 %), principal point (1058, 618), 5-parameter distortion,
   reprojection RMS **0.28 px** on 119 views.
2. **`vidnav/track.py` – measuring the surface.**
   - 1,500 floor features per frame are tracked with sub-pixel optical flow, using a
     forward/backward check (median error 0.03–0.1 px).
   - The static cart wheels and the part of the image the calibration does not cover are
     masked out.
   - SIFT keyframes are stored every 10 frames for re-recognising places.
3. **`vidnav/navigate.py` – position.**
   - **Projection to the floor:** each feature is undistorted, rotated by the camera tilt and
     intersected with the floor plane at height h, which gives metric floor coordinates
     relative to the point directly below the camera.
   - **Camera tilt:** estimated from the video itself as the tilt that makes the floor move
     most rigidly between frames (roll −0.26°, pitch −0.67°).
   - **Motion between frames:** a rigid 2-D motion (RANSAC + least squares) is fitted between
     frame k and k+1, 2, 4, …, 64. Where blur or a bump broke the tracking, sharp frames on
     either side are matched with SIFT instead.
   - **Loop closure:** when the cart returns to a place it has seen before, keyframes are
     matched with SIFT and the match becomes an extra constraint.
   - **Global solution:** all about 16,700 constraints are solved together by robust
     Gauss-Newton on a pose graph.
4. **`vidnav/validate_scale.py` – independent check with the tape measure.** Every frame showing
   the tape is projected to the floor at 0.1 mm/px. The period of the mm graduation is then
   measured by Fourier analysis, separately on the cm edge and on the 台尺 edge.
5. **`vidnav/render.py`** produces the floor mosaic, the plots and the annotated video.

## Outputs (`output/`)

| file | |
|---|---|
| `trajectory.csv` | **position at every frame (60 Hz)**: time, x, y [m], heading, speed, travelled distance, number of constraints, interpolated flag, plus the dead-reckoning and no-loop-closure versions for comparison |
| `trajectory_tape_scaled.csv` | the same, rescaled with the tape-measure scale (see below) |
| `trajectory.png` | path on the floor map, x/y over time, speed, heading, altitude diagnostic |
| `floor_map.jpg`, `floor_map_with_path.jpg` | orthomosaic of the floor at 1 mm/px, built from the estimated poses |
| `position_video.mp4` | camera video plus map with the live position (30 fps) |
| `calibration.png`, `calibration.json` | calibration quality and parameters |
| `scale_check.png`, `scale_check.json` | tape-measure scale check |
| `summary.json` | all key numbers |

Coordinate frame: the origin is the point below the camera at t = 0. X points to the right and
Y to the top of the first video frame.

## Results

- **Path:** a closed rectangle of about 1.0 × 1.95 m, travelled in 47.5 s.
  - Path length is 5.56 m with h = 64.1 cm, or **5.36 m after tape scaling** (computed on the
    path smoothed over 0.25 s).
  - The cart finishes about 38 cm from the start, next to the tape.
- **Internal consistency:**
  - Re-visited places agree to a median of **0.10 mm** after the global solution.
  - Before loop closure, the mismatch had a median of 1.9 mm. Over the full lap, the solutions
    without loop closure deviate from the final path by up to 8 cm (frame-to-frame dead
    reckoning) and 13 cm (multi-span odometry).
  - 95 % of all motion constraints agree within 0.5 mm.
- **Frame-to-frame jitter:** about 0.6 mm RMS, from cart vibration (see limits below). Speed is
  therefore computed over a 0.25 s window.
- **Floor map:** tile joints stay straight and unbroken all around the loop. This is a direct
  visual check of the poses.
- **Only frame 2170** is completely blurred (a bump on a tile joint). Its position is
  interpolated and flagged.

### Important finding: the metric scale

The tape measure in the video is an independent ground truth.

- **cm edge:** with h = 64.1 cm, its 1 mm graduation measures **1.0375 mm** (53 frames; the start and end segments differ by about 0.2 %, which is the realistic uncertainty).
- **台尺 edge:** checked independently (1 分 = 3.03 mm), it gives the same result within 0.2 %.

So all distances computed with h = 64.1 cm are **3.6 % too long**. The tape implies an effective
camera height of **61.8 cm**: everything scales with h divided by the focal length.
`trajectory_tape_scaled.csv` contains the corrected positions.

Likely reasons, in order:
1. The 64.1 cm was measured to a point other than the lens's optical centre, for example the top
   of the phone or the mount. The optical centre sits roughly at the lens glass on the underside.
2. Focus breathing: the focal length changes slightly with focus distance. This explains at most
   about 0.5 % here.

## Limits of accuracy

- **Scale:** h/f, see above. Fixed by measuring the height to the lens and locking focus.
- **Camera vibration:** a 0.1° wobble moves the image like a 1.1 mm translation, and a planar
  scene with a 44° field of view cannot tell the two apart. A drone resolves this with its IMU
  attitude.
- **Floor relief:** granite grain and recessed joints are a few mm high and cause parallax of about
  0.1–0.4 mm between frames. It averages out, but sets the floor of the per-frame noise.
- **Calibration coverage:** the board covered only the centre of the image. Features are
  therefore used only inside the calibrated region, about 58 % of the image.
