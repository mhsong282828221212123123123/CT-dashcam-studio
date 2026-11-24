# New-source review — 2026-11-13

Result: **not accepted**. Three previously unanalysed source timestamps were selected from original footage before predictions. No clip-specific corrections or implementation changes were made for these runs. The existing reconstruction does not generalize adequately to these sources.

Artifacts: `../scene-results/different-source-review/run-20261113/`.
The frozen `configuration.json` records source selection, model/settings, and core source hashes. Each source has raw observations, four-camera diagnostic images/video, fusion audit, MP4 and a review report. `manual-review.json` records failures rather than treating a successful export as reconstruction acceptance.

| Source | Actual speed range in analysed interval | Main observed failure |
| --- | --- | --- |
| 2026-08-22_09-53-09 | 0–59 km/h | Two same-direction lanes become five display lanes. Opposite-road tracks and biased adjacent-car positions vote for extra lanes. At30s two front cars are detected but neither is placed. Abrupt world-position steps also occur at18.6s and34.8s. |
| 2026-09-04_10-50-27 | 44–89 km/h | Three lanes persist after the host enters a single-lane exit ramp around30s. The branch geometry is not reconstructed. Distant front placement is sparse; the current metric fit/range limitations apply. |
| 2026-10-27_18-35-37 | 87–108 km/h | At20s a confirmed front detection has no3D placement. At30s the right car is placed at x=-7.26m outside the two-lane display. |

All runs: fresh YOLO11x640, DirectML, four source cameras,10Hz observations,35s each; no CUDA, OSM attachment or reviewed overrides. Source videos were only read.

Manual comparison covered original/detection/reconstruction at5/20/30s per source, plus both flagged abrupt steps in the first source. The >70m/s world-step check is a review trigger, not a ground-truth accuracy metric; zero flags in the other sources does not establish correct tracking.

Each export decoded1050frames/30fps at960x820 with four originals below. The concatenated `different-sources-4camera-105s.mp4` decoded3150frames/30fps/105s. Chapters:0–35s departure/traffic;35–70s daytime exit;70–105s night tunnel exit.

The new evidence shows why the previous outside-road display filter is insufficient: it runs after lane inference, and requires visual road reference. Without that reference, opposite-road traffic can already create an incorrect corridor. A fixed straight corridor also cannot represent a ramp split. These failures are not resolved by the earlier27s identity-theft fix or a heavier detector alone. Preserve this run as a failed baseline; do not present its videos as corrected/finished output.
