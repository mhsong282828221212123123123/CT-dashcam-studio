# Vehicle motion repair — 2026-11-14

Scope: the 2026-08-19_17-16-37 stopped intersection clip (1..36 s). Road topology code was not extended. Original observations and videos are preserved. No inference was repeated, no CUDA, build, release or push.

The old pipeline fitted every vehicle silhouette with yaw zero before estimating travel direction. Its stopped front horizon also remained at 479.876 px, incompatible with many persistent observed silhouettes. This rejected most moving vehicles before world tracking. A changing occlusion rectangle could then cause a size-based continuation to jump several metres. Constant low acceleration noise also rejected valid turning tracks.

Changes:
- stopped_motion_priors uses a bounded stopped interval, at least six distinct moving car tracks and robust agreement to estimate a shared horizon proxy. Actual sample: 31 track votes, horizon 500.606 px, median absolute deviation 4.653 px. Assumes 1.65 m representative car height; not factory calibration, surveyed distance or host localization. Uncalibrated moving-host footage remains unchanged by this prepass.
- Image ground motion supplies a signed direction prior. Temporally optimized silhouette aspect refines body orientation within 45 degrees; one box cannot flip orientation. Orientation is used in fitting and multi-view reprojection, not just in the final renderer. Forward travel is assumed; reversing is unresolved.
- Oriented footprints replace axis-aligned conflict checks, including postprocessing reprojection.
- Image-size continuation must agree with the existing motion prediction; a broad absolute speed bound alone is insufficient.
- Turning-track process noise reflects speed and bounded body turn rate. Two-sided Kalman smoothing is used only in the supported stopped calibration interval, preserves IDs, and rejects proposed positions that move over 3 m or fail image IoU >= .5; invalid frames split the smoothing into independent validated runs with boundary blending. No lane snapping.
- Display heading changes are bounded to 90 degrees/s after a heading is established, with world-frame compensation.

Verification:
- Final full suite: 288 tests passed in 83.324 s; diff check passed.
- Source identity crops manually inspected at three times for four separate vehicles; see ../scene-results/vehicle-motion-repair/source-identity-check.jpg.
- Current placement count versus previous output: front:22 0 -> 53 of 55 confirmed pixel frames; front:46 29 -> 32 of 32; front:62 7 -> 31 of 31; front:68 1 -> 22 of 27. Each selected vehicle retains one world ID in the repaired output. Counts do not establish surveyed accuracy.
- White turning sedan front:46 maximum frame-to-frame inferred speed: 43.65 -> 9.67 m/s. front:62 initial peak is now 9.95 m/s after splitting image-invalid smoothing runs. Selected-track maxima are 5.61, 9.67, 9.95 and 8.22 m/s. These are consistency checks, not measured ground-truth speeds. Body-versus-motion angle differences remain roughly 15..21 degrees in the selected tracks. This is not ground-truth orientation validation.
- Stopped adjacent vehicles: left_repeater:17 spread X/Z .006/.023 m over 10..30 s; right_repeater:3 .042/.539 m. Residual drift remains on the right vehicle.
- Other cached samples (2026-08-22 turn, 2026-09-04 fast exit, 2026-10-27 tunnel) supply zero eligible correction frames. This checks abstention, not general motion quality on those sources.

Remaining failures: some occluded/far objects are unplaced; the small white car uses 3 interpolated and 2 predicted positions during 5 confirmed-pixel frames; distance bias and body angle errors persist. Multi-camera handoff accuracy and camera calibration are not established by this repair. Intersection road extent and unknown signal states remain inaccurate. This is a measured scoped improvement, not commercial-tool acceptance.

Artifacts: ../scene-results/vehicle-motion-repair/final-motion-validation.json; final2.json; final2-audit.json; vehicle-motion-reviewed-4camera-35s.mp4 (35 s, 30 fps, four original channels below reconstruction). Pickles are locally created debug snapshots only; never load untrusted pickle input.

Final per-track consistency and stationary spread are in ../scene-results/vehicle-motion-repair/delivery-validation.json. Source videos and observations are unchanged.

Renderer identity continuity check (including bounded predictions): selected vehicles are present on all 55/55, 32/32, 31/31 and 27/27 confirmed-pixel frames respectively. front:22 includes 3 reconstructed and 2 predicted frames; front:68 includes 3 interpolated and 2 predicted frames. This is distinct from direct source-linked metric placement counts above.
