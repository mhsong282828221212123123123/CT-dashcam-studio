# Front-track continuity repair

Display-policy update: see SCENE_PRESENCE_RENDERING.md. The position-state names below remain, but `pitch_held`/`image_supported` no longer imply a translucent body when current camera evidence confirms the vehicle. Ghost appearance now represents missing presence; position uncertainty remains separately recorded.

The8/22 09:53:09 source kept confirmed pixel IDs front:8 and front:2 while their3D placements disappeared around24–31s. The single-boundary pitch proxy moved from approximately471px to404px, grounded-box fits failed, and world tracks expired after0.8s despite continuing image evidence.

## Final implementation

- Preserve existing road projection/pitch behavior. A global pair-only pitch change was tried and rejected after changing other clips' lane counts. Keep an independent recovery camera whose pitch updates only from paired lane evidence.
- If ordinary front fitting fails with weak pitch, retry a sufficiently large box using the recovery camera. Limit recovery to45m. Such poses are `pitch_held` ghosts, not observed lane-vote or motion-heading anchors. Candidate-specific camera geometry is retained for joint fitting and image-anchor creation.
- A confirmed front image ID can lease an established near-world identity. Shape/scale and horizontal bearing continue position from the last accepted image/metric anchor, ignoring unreliable vertical ground placement. These positions are `image_supported` ghosts. This is a cheap box-size/bearing calculation, not additional neural inference.
- Require an existing unclipped front anchor within45m. Limit anchor-based continuation to6s, range65m, width/height scale agreement1.35, scale ratio0.4–2.5 and a bounded world step. Occlusion slivers cannot become rapidly receding/flying cars. New image-only detections cannot seed metric vehicles.
- Use the latest supported position for reacquisition gating, keeping the existing protection against one local ID stealing another vehicle. A stale leased identity cannot claim a fresh cross-camera observation.
- Short image dropouts after an image-supported estimate use bounded prediction, then expire normally. Rear support cannot lease identity. Existing side-camera and distant-vehicle memory are unchanged. Boundary-excluded/out-of-sync/unconfirmed detections provide no new support.

`pitch_held` and `image_supported` are explicit estimates. Display uncertainty is conditional/heuristic and does not establish absolute position accuracy. Position/road calibration, all far-new-object failures, general junction topology and unmarked median recognition are not solved by this repair.

## Validation

Artifacts: `../scene-results/front-track-continuity-review/`.

- Final main source: front:8 remains fused:15 and front:2 remains fused:3 in every one of91frames from23.0 through32.0s. No missing display frames.26–27s use ghosts;27.4s includes an actual wiper-caused detector dropout but the existing ID remains as a bounded ghost. No >70m/s world-step review flags across the source.
- Other originals:9/4 three-lane approach stays3;10/27 night stays2; original user50s stays2 with maximum lateral-phase difference0 and no near false ghost at27s. No >70m/s review flags in these final runs. The motion threshold is an anomaly trigger, not a ground-truth accuracy metric; known ramp geometry/absolute-position limitations remain.
- Full270tests passed113.778s. New regressions cover continuous2D identity across metric failure/reacquisition, short dropout then expiry, no image-only spawning, rear exclusion, sliver rejection, unchanged side/far memory, paired recovery pitch and ghost recovery under an invalid single-line pitch.
- `final-regression-summary.json`, per-source `*-final-audit.json` and `final-code-hashes.json` retain evidence. Earlier global-pitch/scoped experiments in the same folder are superseded.
- `front-track-continuity-4camera-35s.mp4` decoded1050frames/30fps/960x820 with four original views below. Exported26.0/27.4/30.0s frames were visually compared against the sources. `full_scene_quality_accepted` remains false.

After restarting Scene Lab, opening an existing observation result rebuilds this timeline without rerunning object inference. Source videos and original observations were not modified. No build/release/push was performed.
