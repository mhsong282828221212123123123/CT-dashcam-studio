# Vehicle-in-lane prior — 2026-11-14

Normal same-direction traffic is assumed to occupy a lane. Consistent image
evidence can release that assumption for lane changes, crossing traffic and
turns. This is a soft constraint, not a claim of measured lane geometry.

## Corrected omissions

The vehicle-supported corridor builder only accepted `tracking_state=observed`
and a short list of COCO types. Consequently image-supported whole-track
reconstructions, SUVs, vans and open-bed trucks could be visible in the scene
but unable to establish their lanes. The reviewed two-lane sample was drawn
as one lane.

The builder now includes all supported vehicle subtypes and currently
image-supported reconstructed poses. Unverified position hypotheses, ghosts
without current observations, opposite traffic and crossing/turning bodies
do not establish new same-direction lanes. Stable residence remains required;
one noisy position does not create an extra lane. Road-scope filtering shares
the same vehicle-type vocabulary.

## Soft containment

`core/scene_lane_prior.py` establishes membership from sustained same-lane
observations. For reliable camera-referenced road intervals, a footprint that
slightly overlaps a boundary gets a bounded lateral correction toward the
lane interior. A normally off-centre car stays off-centre. It is not snapped
to the middle, and its heading/forward position in road coordinates are not
used as free variables to hide a camera error.

* Lane changes with coherent lateral motion, turns, active junction geometry,
  unsupported road references and large conflicts release the correction.
* Vehicle-only inferred road geometry can establish a display lane; it cannot
  independently certify a corrected position of that same vehicle.
* Per-vehicle correction is capped at 0.35 m and backtracked if the original
  camera projection deteriorates excessively (IoU drop >0.10 or IoU <0.45).
  Raw positions and applied corrections are retained in the audit.
* Offline smoothing only reduces corrections that already passed the gates.
  It anticipates release and keeps zero-support frames at zero. Correction
  magnitude changes are limited to 0.35 m/s, preventing a new lateral jump when
  paint disappears or the prior switches off. Projections are checked again.
* No host odometry, host lane phase, OSM outline or lane count is changed by
  the containment step. Lane count comes from the preceding evidence builder.
* Containment uses the road reference frame (`road_scope_view`), not the camera's
  display-follow rotation. Changing the display viewpoint cannot change the
  constraint. Reconstructed turn-path geometry releases the straight-corridor
  prior. The `postprocess=False` analysis option bypasses position correction.

## Same-source replay

Source: `2026-10-15_17-33-13`, the existing 35-second queue/start sample.
Original observed lane segments were restored before rebuilding the road.
No detector, model weights or source media were changed for this replay.

* Display lane count: **1 -> 2**, throughout the sample.
* Outer boundaries solid; the internal separator dashed.
* Small bounded corrections affected the observed right-side sedan and the
  rear-approaching truck after its entry turn, not the host.
* Host odometry arrays were checked and remained identical.
  Host display translation and yaw were also checked after replaying the normal
  heading-display stage, and remained identical.
* Existing motion-outlier audit: 0 before / 0 after. This is a discontinuity
  check, not an independent motion-accuracy measurement.
* An initial implementation introduced correction-switch jumps (up to
  1.75 m/s of artificial lateral motion). It was rejected and smoothed. The
  final sample's correction-change maximum is 0.35 m/s for both affected tracks.

Regression checks cover reconstructed/subtype lane support, rejection of
cross traffic/hypotheses, off-centre driving, real lane changes, local curved
road coordinates, image conflicts, unchanged host position, and smooth release
when paint is lost.

267 scene tests passed, including display-camera rotation invariance.

Artifacts: sibling `scene-results/current-sample-20261114/lane-prior-review/`
contains `report.json`, `audit.json`, `timeline.pkl`, and `proof.jpg`.

```powershell
python tools/review_scene_lane_prior.py ../scene-results/current-sample-20261114
python -m unittest discover -s tests -p test_scene*.py -q
```

The camera-body calibration and absolute vehicle distances are still unverified.
This change does not make the reconstruction metrically accepted.
