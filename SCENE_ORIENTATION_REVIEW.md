# Queue-sample orientation review — 2026-11-14

The source is `2026-10-15_17-33-13`, 1–36 seconds. The preceding MP4 was checked
again against original front, back and right-repeater frames.

## Findings

* Right-front SUV (`front:1`) accumulated about -9.4 degrees at 12 s and -11.3
  degrees at 24 s while stationary. Axis-aligned box agreement had allowed
  body angle and dimensions to compensate for each other. Good box IoU is not
  independent evidence of body orientation.
* Rear-approaching vehicle (`right_repeater:9`) is an open-bed Porter-type truck,
  not a box van. It also turns into the road in the 12–16 s source interval.
  Its approximately 53-degree early estimate cannot be declared wrong merely
  because it differs from straight ahead. The original rear image was inspected
  as well as the repeater image; the real turn must be preserved.
* Rear imagery is currently association/reference evidence, not a calibrated
  source of 3D position or yaw. Showing four original views does not establish
  four-camera metric calibration. The side truck pose remains conditional on
  the estimated repeater extrinsics.

## Changes

* For an established stationary run lasting at least a second, the bundle fit
  holds the input orientation in world coordinates. A rectangle alone must not
  turn a stationary body to conceal shape mismatch. This retains a prior; it
  is NOT an automatic measurement of the absolute facing direction.
* If an image-supported candidate was lost by later position filtering, its
  original orientation prior is retained for reconstruction. Unknown values
  are not silently replaced with a measured zero direction.
* Explicit truck geometry labels now take precedence over automatic
  box-dimension shape selection.
* The reviewed sample labels the open-bed truck as `small_truck`. Its projection
  envelope has a low rear bed and a high cab at +Z, matching the rendered asset's
  layout. A full-height cargo box no longer substitutes for the open bed.
* No source boxes, lane positions or 3D trajectories were manually moved.
  Subtype labels remain explicitly assisted by visual review.

## Same-source results

| Source time | Right-front SUV yaw | Approaching truck yaw |
| --- | ---: | ---: |
| 12 s | -0.01 deg | 54.77 deg, entering turn |
| 16 s | -0.01 deg | 3.25 deg |
| 24 s | -0.01 deg | -0.01 deg, stationary |
| 28 s | 0.03 deg | 0.00 deg, stationary |

The SUV dimensions also ceased compensating for the false angle: fitted length
changed from 4.31 m to 4.93 m. Those are bounded model estimates, not measurements.
The truck height fit still reaches its allowed upper bound (2.46 m): its absolute
size, location and camera-body calibration remain unresolved. This update is not
a claim of finished metric reconstruction.

Regression tests cover held stationary heading despite a misleading box aspect,
conversion of a local prior to world orientation, open-bed front/rear projection,
and preservation of explicit truck shape evidence. 258 scene tests passed.

A trial feature-flow direction estimator was not adopted: it rejected most early
turn/occlusion observations. Its diagnostic JSON files are retained locally;
they do not constrain the production poses.

Outputs: sibling `scene-results/current-sample-20261114/`, `build-v3.log`,
`current.pkl`, `current-models.json`, `current-audit.json`, and the v3 MP4.
