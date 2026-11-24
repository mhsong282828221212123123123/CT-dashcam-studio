# Camera-zero investigation — 2026-11-14

## Result and limits

The current HW3 defaults are estimated priors, not measured camera calibration.
No publicly accessible, traceable calibration table for this specific 2020
Model 3 LR's recorded 1280x960 images was obtained. Searching local project
filenames for camera/calibration/intrinsic/실측/측정 also found no surveyed input.
This is not a claim that no such data exists anywhere.

The front optical model has an empirical, held-out reprojection check.
An initial side-camera rigid fit failed. The subsequent timestamp-aware
experiment passed 14 of 16 held-out images, so the overall gate still fails.
No new intrinsic/extrinsic values are applied to app defaults, and metric
vehicle placement is NOT verified. Actual frame sampling and numeric odometry
have been corrected in the app's shared analysis path.

## Public-source review

* Tesla's [2017–2023 Model 3 dimensions](https://www.tesla.com/ownersmanual/2017_2023_model3/en_us/GUID-56562137-FC31-4110-A13C-9A9FC6657BF0.html)
  give body dimensions and wheelbase (2875 mm), not optical-center coordinates,
  focal lengths or lens distortion coefficients. Body dimensions cannot supply
  the missing camera extrinsics by themselves.
* [SAE 2023-01-0629](https://saemobilus.sae.org/papers/evaluation-tesla-dashcam-video-system-speed-determination-via-reverse-projection-photogrammetry-2023-01-0629)
  actually calibrated a 2019 Model 3's front/rear cameras. The accessible
  abstract describes the method, but does not expose its calibration tables.
  No numbers have been inferred from the abstract or a paywalled paper.
* [NATIX dataset](https://huggingface.co/datasets/natix-network-org/natix-multi-camera-driving-dataset)
  advertises multi-camera calibration metadata. Download requires an approved
  access request/account information sharing. No gated files were retrieved.
  Its nominal FOV is not a calibration of this user's car.
* [Public DPVO Tesla config](https://github.com/OVR-Platform/DPVO/blob/main/calib/tesla.txt)
  is only an unverified third-party reference; hardware, crop and measurement
  provenance are insufficient for installation as a measured HW3 profile.
* [Tesla SEI schema](https://github.com/teslamotors/dashcam/blob/master/dashcam.proto)
  encodes speed as floating-point m/s. The decoder now retains this value in
  `speed_mps`; display `speed_kmh` remains rounded. Calibration/odometry must
  not discard low-speed travel by integrating the rounded display value.

## Actual-image experiment

Source: `E:\tesla\a3\TeslaCam\RecentClips\2026-08-15_17-39-32-front.mp4`
and synchronized left/right/back siblings; original files unchanged.

CPU pycolmap 4.2.1 was installed into a separate sibling runtime directory.
Static image features were matched, with detected traffic regions masked.
Unconstrained SfM reconstructed 22 front images and a separate 17-image
left/right component. Front and side components did not join. Rear did not
register. Thus this is NOT a calibrated four-camera rig.

### Front optical check

Frames at 4, 12 and 20 seconds were removed from bundle adjustment. Each held-out
image was localized using two thirds of its surviving 3D point correspondences;
the other third supplied the reported pixel errors. Retained 3D points require
at least three training observations. All points are within the same recording;
this does not establish accuracy on another day or a metric distance bound.

Estimated SIMPLE_RADIAL `[f,cx,cy,k1]` after this exclusion:
`[1634.3442,640,480,-0.3688346]`, at 1280x960.

| Held-out time | Unused test points | Median error | 95th percentile |
| --- | ---: | ---: | ---: |
| 4 s | 138 | 0.62 px | 2.35 px |
| 12 s | 81 | 0.42 px | 1.87 px |
| 20 s | 64 | 1.11 px | 3.00 px |

These are optical reprojection results, not metre errors. The selected 3 px
threshold is a development gate, not a manufacturer's specification. A legacy
850 px, zero-distortion optimization diverged from the new reconstruction's
initialization; its numbers are invalid as a quantitative comparison.

### Left/right rigid check — rejected

Both images at 22 seconds were excluded before rigid bundle adjustment.
The left image alone was localized. The right pose was predicted using the
trained rig transform; right image pixels did not optimize that pose.

106 right-side test points: median **6.86 px**, p95 **13.48 px**, max **90.40 px**.
The gate was median <=3 px and p95 <=8 px: **FAIL**. Scale is arbitrary, and
the left/right translation is NOT a measured baseline. A small training error
(0.91 px) alone would have concealed this failure.

## What remains before applying a profile

1. Resolve the side rig's held-out error and join front/side static geometry.
2. Establish metric scale and optical-center/body-frame origin from independent
   physical references; body dimensions alone do not establish camera positions.
3. Validate on a separate easy clip using original-image reprojection and
   cross-camera transitions. Do not use vehicle-box fit as metric ground truth.
4. Apply focal and distortion together. Updating focal alone while detections
   remain in distorted pixels would introduce another inconsistent coordinate
   space. Existing old detections require transformation or fresh inference.

Artifacts under sibling `scene-results/camera-zero-review/rig-sfm/`:
`front-heldout-validation.json`, `front-heldout-proof.jpg`,
`heldout-022.0/validation.json`, database and sparse reconstructions.

Reproduction (from repository root):

```powershell
python tools/validate_scene_sfm_intrinsics.py ../scene-results/camera-zero-review/rig-sfm
python tools/validate_scene_sfm_rig.py ../scene-results/camera-zero-review/rig-sfm
python -m unittest discover -s tests -p test_scene*.py -q
python -m unittest discover -s tests -p test_decoder_speed_precision.py -q
```

Validation run: 243 scene tests + 2 decoder precision tests passed. Python tests
check code behavior; they do not change the side rig's failed real-video result.

## Follow-up implementation and validation

### Corrections applied to analysis/export code

* `CameraReader` selects the closest actual MP4 presentation timestamp instead
  of treating average FPS as a per-frame clock. The source strip in MP4 exports
  uses this same reader. PyAV indexes packet timestamps without decoding all
  images. Missing/ambiguous PTS explicitly falls back to unverified nominal FPS.
* Analysis rows retain the actual selected timestamp and requested-time error.
  The manifest records the timing method for each camera.
* SEI `frame_seq_no` is retained and its cross-camera correspondence is audited.
  **It does not override PTS.** Tests with actual samples showed that a common
  metadata counter does not establish simultaneous camera images. The initial
  sequence-only alignment hypothesis was rejected for operational sampling.
* `WorldTracks` and ego wheel travel now prefer floating-point `speed_mps` over
  rounded display speed. At 0.08888864 m/s, six seconds now contributes 0.5333 m
  rather than zero. This fixes numeric integration, not wheel-speed sensor error.
* Per-camera lens files are now supported before fresh inference/tracking,
  including front/side cameras. Raw reviewed boxes cannot be mixed with
  rectified pixels. CLI: `--camera-calibration front=PATH_TO_LENS_JSON`.
  Estimated profiles have not been installed as defaults or exposed as a
  claim of completed metric calibration.

### Real-file timing audit

Three different recordings, 340 requested times per recording, all four cameras.
Numbers below are 95th-percentile absolute differences between requested times
and **container presentation times**, not measured sensor exposure errors.

| Recording / left repeater | Average-FPS sampling | New PTS sampling |
| --- | ---: | ---: |
| 2026-08-15 17:39:32 | 30.7 ms | 14.6 ms |
| 2026-10-15 17:33:13 | 73.0 ms | 14.4 ms |
| 2026-09-16 22:15:39 | 24.2 ms | 14.4 ms |

Full per-camera and inter-camera results: sibling
`scene-results/camera-zero-review/timing-audit.json`.

### Denser side-camera experiment

37 synchronized-input samples per camera, 0.5 s spacing, 9–27 s, 1280x960,
4000 SIFT features, RADIAL (two radial terms), CPU. SfM reconstructed a separate
53-image side component (37 left, 16 right) with mean training error 0.50 px.
Front remains a separate component; rear is not calibrated.

Actual PTS differences were then included using a continuous pose trace, with
a bounded constant clock-offset parameter. For each query, both query images
were removed before bundle adjustment; the left image alone localized the
query pose. Right query pixels never optimized the predicted query pose or
rig transform. Relative camera motion over the PTS difference came from the
training-only left pose trace. Held-out 3D points require at least three
training observations. Pose RANSAC uses a fixed seed and a 3 px inlier threshold.

All 16 available right images (19.0–26.5 s) were checked individually, with the
original median <=3 px / p95 <=8 px thresholds. **14 pass, 2 fail.**

| Held-out image | Test points | Median | p95 | Result |
| --- | ---: | ---: | ---: | --- |
| 19.5 s | 88 | 4.47 px | 6.75 px | FAIL |
| 21.0 s | 239 | 2.84 px | 4.54 px | PASS |
| 22.0 s | 201 | 4.42 px | 5.96 px | FAIL |
| 23.0 s | 228 | 0.98 px | 2.56 px | PASS |

The earlier 21 s rigid test gave p95 16.21 px. The new 21 s result is 4.54 px.
This comparison combines denser sampling, a different distortion model and
timestamp compensation; it does not attribute all improvement to one change.
The 16 tests reuse one recording, not 16 independent journeys. Some frames have
few test points (minimum 21), and points are mostly background landmarks.
These pixel results do not verify foreground vehicle distance or body origin.

The 2 failing frames remain failures. Further lens-parameter fitting did not
remove the 22 s bias and was not adopted. Small pixel residuals alone cannot
establish metre scale, optical-center heights or vehicle-frame coordinates.

Artifacts: `scene-results/camera-zero-review/rig-sequence-sfm/` contains
`input-index.json`, `sync.json`, sparse models,
`timed-all-frames-validation.json`, and per-query point overlays/NPZ projections.
The two-point front reconstruction with nonphysical lens parameters is rejected;
it must not be treated as a calibration just because its training error is low.

Reproduce an individual timestamp-aware test:

```powershell
python tools/validate_scene_timed_rig.py ../scene-results/camera-zero-review/rig-sequence-sfm --time 022.0.jpg --fit-offset
```

Full test run after the main changes: **319 tests passed**. A subsequent optional
PTS-dependency fallback test was added and checked with the related test suite.
The app's changed behavior is frame selection, telemetry precision and support
for applying explicit lens files. A corrected metric 3D reconstruction/video is
still outstanding; no release or build was made.

### Known-size reference in a separate easy recording

An explicit manually marked plate-plane reference was added for the stationary
Carnival in `2026-10-15_17-33-13`, independent of the August calibration recording.
The [official plate specification](https://www.law.go.kr/admRulLsInfoP.do?admRulSeq=2100000256300)
lists 520x110 mm for the visually matching long-format plate type. This is a
standard-size assumption, not a personal measurement of that physical plate.
No OCR is performed.

Using the inferred front lens model, planar PnP estimates its centre at camera
coordinates `(0.021, 0.420, 6.744)` m (OpenCV convention: right/down/forward).
The same corners under the old 850 px zero-distortion model give 3.515 m depth.
These are two conditional estimates, not proof from a tape-measured distance.
They demonstrate how strongly the old focal assumption affects metric scale.
Neither number is a vehicle-centre coordinate or bumper-to-bumper gap.

The marked plane was tracked over 13 stationary frames (12–24 s). Each tracking
homography was checked on excluded image features; all 13 passed its 1.5 px
p95 image-registration gate. The near-identical frames are not independent
distance measurements; tiny depth variation must not be sold as millimetre
accuracy. With assumed 0.5 px corner noise and 2% focal noise, sensitivity spans
6.458–6.992 m. This interval excludes unknown systematic lens/type/boundary bias
and is NOT an absolute confidence bound.

Review artifacts: `scene-results/camera-zero-review/plate-anchor/validation.json`
and `reference-proof.jpg`. The original image with the projected plate plane and
axes was visually inspected. The plate anchor is a separate reference tool;
it does not automatically reposition detected cars in the main app.

```powershell
python tools/validate_scene_plate_anchor.py
```
