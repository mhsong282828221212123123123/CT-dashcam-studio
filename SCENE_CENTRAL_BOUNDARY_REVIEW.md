# Visible central boundary and junction review — 2026-11-13

The user requested a cheap separator-first rule rather than more complicated motion classification. The primary gate now uses colour/Hough line detection on640px camera images. It recognizes visible yellow longitudinal paint, including the yellow edge below a median in the reviewed source. It does **not** recognize every unmarked concrete/metal median.

The detected line divides image space into host side and far side. Both bottom corners of a vehicle box must be beyond the line with a margin over repeated observations before exclusion. Established host-side tracks and straddling vehicles are preserved. If the line disappears, it is not extrapolated through the next frames/junction. Uncalibrated distorted rear images are not used for this gate.

`road_relation` and `central_boundary` evidence are stored with observations. Far-side detections are rejected before metric fitting, fusion and lane voting, while raw detector output remains available. New analyses include this automatically. `tools/enrich_scene_central_boundary.py` produces a NEW enriched cache from original videos without rerunning the detector or modifying source/cached observations; rectified caches require reanalysis rather than mixing coordinate systems.

With no valid paint phase, lane voting uses each camera-local source's first stable near-field window. A drifting reconstruction of the same source cannot establish multiple lanes. Nearby lane-position hypotheses are grouped only for lane counting; actors/ego are never snapped to lane centres. Major-turn/crossing samples do not vote for extra longitudinal lanes. Tests preserve genuine three-lane layouts rather than imposing a two-lane cap.

For the controlled turn, a temporally observed horizontal signal, substantial vehicle-heading turn and missing longitudinal paint jointly support a junction hypothesis. A turn alone is insufficient. Parallel markings are interrupted in an estimated open junction surface. Exit paint/heading stability starts a new road segment; lateral phase is not inferred from paint inside the junction. This is not general verified junction/branch topology and does not resolve every exit ramp.

## Actual source evidence

Artifacts: `../scene-results/central-boundary-review/final/`.

- 8/22 09:53:09: the former five-lane display becomes two lanes throughout350frames. Junction interval10.0–18.2s,82inside frames; exit is a separate road segment.625 far-side detection instances (205front,420left repeater; not625 unique cars) are excluded. Source reviews5/13/20/30s confirm own lead/right sedan/right truck remain and visible traffic beyond the centreline/median edge is excluded. Counter-traffic in an unmarked junction is not erased by an invisible extended line.
- 9/4 10:50:27: three-lane approach remains three lanes; no forced two-lane cap. The existing later single-lane ramp topology failure remains unresolved. Only one active front boundary frame, so this gate conservatively makes no exclusions there.
- 10/27 night: two-lane display retained;7 far-side left-repeater observation instances excluded. Vehicle-placement omissions remain.
- Original user50s clip: two lanes retained, maximum lateral-phase difference0, no near ghost at27s. No useful yellow-line evidence in this clip, so the new separator gate abstains; it does not claim to detect its unmarked median.

Full262tests passed111.760s. Targeted rectification5 and central-boundary5 tests repeated after the calibration-coordinate ordering safeguard and person-preservation guard. Synthetic tests also cover signal-free curves, junction marking interruption, exit road reset, source drift and true three-lane occupancy.

Output `centre-boundary-junction-4camera-35s.mp4` decoded1050frames/30fps/960x820 with four original cameras below. Source images and frame-level/fusion audits are in the per-source folders; `review-summary.json` records the four cases and `code-hashes.json` records implementation hashes. Overall scene quality remains unaccepted: misses/lateral placement bias, exact junction outlines, unmarked median recognition and some ramp topology remain open.
