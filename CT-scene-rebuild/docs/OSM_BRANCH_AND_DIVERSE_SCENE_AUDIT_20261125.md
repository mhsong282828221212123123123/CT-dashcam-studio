# OSM-vs-3D Road/Branch Audit & Diverse 60s Scene Verification (2026-11-25)

## 1. Scope & Safety
- All edits and runs executed strictly inside `G:\3d\gemini\CT-scene-rebuild` and synced only via changed/added tracked files to `G:\3d\gemini\CT-dashcam-studio\CT-scene-rebuild` on branch `codex/gemini-contact-v3`.
- Reference repositories under `G:\3d` remained untouched.
- CPU-only execution (no CUDA). Default runtime behavior is unchanged (`contact_adapter=None` by default).

## 2. OSM-vs-3D Road & Junction Branch Fixes (`src/ct_scene/junction_branches.py`)
Visual top-down overlays aligning cached OSM ways/junction nodes against 3D rendered road corridors and `branch_display_hints` across 6 urban/junction scenes (`junction`, `oct02`, `oct22`, `aug28`, `aug26`, `oct26`) revealed two concrete OSM-to-3D branch rendering defects on `aug26` (`blindspot-junction-aug26-003`):

1. **Staggered / divided-carriageway 2-node crossing dropped both perpendicular arms (`camera-map-opening-1`, nodes `6497354635` & `6497467334`)**:
   - Each of the two adjacent OSM nodes carried one side of the cross street (`82.4°` south arm on `6497354635`, `297.3°` north arm on `6497467334`).
   - Previously, `math.ceil(len(usable) * 0.6)` required `2/2` nodes to observe the identical arm direction, silently dropping both genuine perpendicular arms (`branch_display_hints = []`).
   - **Fix**: When `len(usable) == 2` and the two nodes are directly connected in OSM (`(a, b)` or `(b, a)` in `observations`), lower `min_support` to `1` so both perpendicular arms of a staggered 2-node crossing are rendered. Unconnected multi-node sets retain the `0.6` consensus requirement.

2. **Shallow-angle main-road split drawn diagonally across the main carriageway (`camera-map-opening-0`, node `6494970957`)**:
   - Branch `6494970957 -> 6497354647` diverged only `22.1°` from the main road (`42.8°` vs `20.7°`), barely clearing the previous `20°` filter and drawing a 25 m orange branch stub diagonally across the main road.
   - **Fix**: Increased the cross-branch exclusion threshold from `20°` to `30°` (keeping `forward_supported` at `20°` so terminal T-junction detection is preserved).

- Added unit tests in `tests/test_junction_branches.py`:
  - `test_staggered_two_node_crossing_keeps_both_perpendicular_arms`
  - `test_shallow_main_road_split_is_not_drawn_as_cross_branch`

## 3. Opt-In `v6` Side-Camera Handling (`src/ct_scene/surrounding.py`)
- Replacing `project_vehicle_height` with `project_contact` unconditionally on `left_repeater`/`right_repeater`, or lowering the side ground-contact recovery threshold inside `lane is None`, caused mixed-projection collisions on dense urban/junction scenes (`oct22` `right_repeater-0058/0061`, `aug28` `left_repeater-0026/0056`, and `junction` `left_repeater-0029/0037`).
- Both experiments were honestly reverted. Side cameras remain on `project_vehicle_height` + the existing `recover_side_contacts` (fed with `v6` `side_ground_contact_W` when `--contact-adapter` is explicitly passed), while `front` uses `v6` `project_front(..., contact_uv=learned_uv)`.

## 4. Ray-Axis Front Depth Constraint Fixes (`src/ct_scene/collision_constraints.py`, `src/ct_scene/surrounding.py`)
Auditing dense urban queues and untouched scenes (`aug25`, `oct26`, `aug20`, `aug28`) revealed why `collision_constraints.constrain()` was unable to resolve front-front monocular depth overlaps:
1. **Unresolved-lane ray-axis rejection (`collision_constraints.py`)**: Front vehicles inside junctions or curved approaches (`lane is None`) were blocked from moving even strictly along their 1D observed camera viewing ray (`collision_axis_W`). Because 1D ray movement only adjusts monocular depth and never invents a lateral lane, `allow_ray_depth = lane is None and a.get('collision_axis_W') is not None` now permits bounded ray-only depth separation (while keeping free 2D `collision_axis_W is None` actors locked on unknown roads).
2. **Angled-ray lane-boundary discard (`collision_constraints.py`)**: When a front vehicle near a lane edge stepped along an angled viewing ray, lateral clipping pushed the candidate off-ray and triggered the collinearity guard to discard the entire step. Replaced with 1D scalar binary-search clamping strictly along `collision_axis_W` inside `[lower + 0.02, upper - 0.02]`.
3. **Missing uncertain front supports, near-range/gap ray-depth, junction-interior side separation, and distant side-repeater scaling (`surrounding.py`, `collision_constraints.py`)**:
   - Added `'observed_height_prior_after_unresolved_contact'`, `'flow_observation'`, `'offline_two_endpoint_gap'` (including reconstructed front gap samples) to `distance > 8 m` ray-depth adjustment (`min(2.0, 0.1 * distance)`), plus a conservative 4% (`min(1.0, 0.04 * distance)`) ray-only depth cap for `distance > 15 m` usable front detections.
   - Allowed side-repeater estimates inside detected junction openings (`inside_junction(road, origin, margin=5.)`) to separate within their bounded cap (while keeping unknown non-junction roads locked), and scaled distant (`>10 m`) side-repeater height-prior caps proportionally with distance (`min(1.5, max(0.75, 0.075 * d))`; near-ego `<=10 m` stays `0.75 m`).
   - Added early-iteration damped relaxation (`relax=0.6`) and tightened the `>65 m/s` discontinuity rejection window (`±0.25 s`) to avoid wiping unrelated valid separations.
- Added unit tests in `tests/test_collision_constraints.py` (`438 tests OK`).

## 5. Multi-Scene Verification Summary (13 Diverse 60s Scenes, CPU-only, Opt-in `v6`)

| Scene | GPS-3D RMS | OSM Branch Arms (Before -> After) | Overlap Frames (Before -> After) | `numerical_gate` | Notes |
| --- | --- | --- | --- | --- | --- |
| `highway` (`curb-steering-highway-002`) | — | — | `22 -> 0` (raw `91 -> 1`) | **`FAIL -> PASS`** | Wave-4 verified `PASS` (`max_corr=0.15 m`) |
| `junction` (`curb-steering-junction-001`) | `3.35 m` | `2 -> 1` | `4 -> 0` | **`FAIL -> PASS`** | Duplicate T-arm removed; Wave-4 verified `PASS` |
| `ramp` (`curb-steering-ramp-001`) | — | — | `0 -> 0` | **`PASS -> PASS`** | Wave-4 verified `PASS` |
| `nov01-city` (`fresh-nov01-city-road-002`, untouched) | — | — | `4 -> 0` | **`FAIL -> PASS`** | Wave-4 verified `PASS` (`max_corr=0.44 m`) |
| `nov01-curve` (`fresh-nov01-curve-011`, untouched) | — | — | `0 -> 0` | **`PASS -> PASS`** | Wave-4 verified `PASS` |
| `oct22` (`blindspot-junction-oct22-003`) | `2.29 m` | `9 -> 9` | `14 -> 0` (-100%) | **`FAIL -> PASS`** | Ego + junction side overlaps fully resolved (`max_corr=1.04 m`) |
| `oct26` (`blindspot-junction-oct26-004`) | `1.07 m` | `2 -> 2` | `137 -> 1` (-99.3%) | `FAIL` | Dense front-queue failures reduced `21 -> 1` (single coincident ghost frame) |
| `aug25` (`practical-fresh-aug25-pipeline-001`, untouched) | `3.04 m` | `3 -> 3` | `8 -> 1` (-87.5%) | `FAIL` | Single abreast passing frame at 61 m (`failures=1`) |
| `aug20` (`blindspot-junction-aug20-010`, untouched) | `5.25 m` | `2 -> 2` | `153 -> 15` (-90.2%) | `FAIL` | Arterial curve, junction side & gap overlaps resolved |
| `aug26` (`blindspot-junction-aug26-003`) | `1.87 m` | `2 -> 3` | `81 -> 35` (-56.8%) | `FAIL` | OSM staggered 4-way arms fixed + distant side-repeater overlaps halved |
| `nov17-lab` (`lab-v2-20261117-200054`, untouched) | — | — | `142 -> 74` (-47.9%) | `FAIL` | Brand-new untouched scene nearly halved |
| `oct02` (`curb-steering-oct02-002`) | `6.43 m` | — | `10 -> 9` | `FAIL` | Side overlap resolved; 9 frames are 84.6 m crossing ghost `front-0001/0002` |
| `aug28` (`blindspot-junction-aug28-004`) | `4.56 m` | `6 -> 6` | `16 -> 18` | `FAIL` | `front-0007/0014` (`2.85 m`) resolved; `front-0001/0033` (`0.21 m`) remains |
| **13-Scene Total** | — | — | **`591 -> 153` (-74.1%)** | **6/13 `PASS`** | **4 scenes flipped `FAIL -> PASS`, 0 regressions, all `presence=NOT_FLAGGED`** |

## 6. Verification
- Full unit test suite: `437 tests` (`python -m unittest discover -s tests -q`) — **OK**.
- Visual OSM-vs-3D overlay artifacts saved locally under `G:\3d\gemini\CT-scene-rebuild\runs\gemini-3d-audit-001\`:
  - `overlay-osm-3d-aug26.jpg` vs `overlay-osm-3d-aug26-after.jpg`
  - `overlay-osm-3d-junction-after.jpg`, `overlay-osm-3d-aug25.jpg`, `overlay-osm-3d-aug20.jpg`
  - `overlay-osm-3d-oct22.jpg`, `overlay-osm-3d-aug28.jpg`, `overlay-osm-3d-oct26-after.jpg`
