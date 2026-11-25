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

## 4. Multi-Scene Verification Summary (8 Diverse 60s Scenes, CPU-only)

| Scene | GPS-3D RMS | OSM Branch Arms (Before -> After) | Overlap Frames (Before -> After) | `numerical_gate` | Notes |
| --- | --- | --- | --- | --- | --- |
| `highway` (`curb-steering-highway-002`) | — | — | `22 -> 0` (raw `91 -> 1`) | `FAIL -> PASS` | Opt-in `v6` front contact |
| `junction` (`curb-steering-junction-001`) | `3.35 m` | `2 -> 1` | `4 -> 0` | `FAIL -> PASS` | Duplicate T-arm removed; ego overlap & gap pinch fixed |
| `ramp` (`curb-steering-ramp-001`) | — | — | `0 -> 0` | `PASS -> PASS` | Preserved |
| `oct02` (`curb-steering-oct02-002`) | `6.43 m` | — | `10 -> 10` | `FAIL` | 9 frames are 84.6 m distant ghost `front-0001/0002` |
| `oct22` (`blindspot-junction-oct22-003`) | `2.29 m` | `9 -> 9` | `14 -> 3` | `FAIL` | All 11 `__ego__` vs `left_repeater-0035` overlaps resolved |
| `aug28` (`blindspot-junction-aug28-004`) | `4.56 m` | `6 -> 6` | `16 -> 21` | `FAIL` | `front-0007/0014` (`2.85 m`) resolved; `front-0001/0033` (`0.33 m`) +8 frames |
| `aug26` (`blindspot-junction-aug26-003`) | `1.87 m` | `2 -> 3` | `81 -> 83` | `FAIL` | Shallow `22°` main-road arm removed; staggered 4-way north/south arms recovered |
| `oct26` (`blindspot-junction-oct26-004`) | `1.07 m` | `2 -> 2` | `137 -> 138` | `FAIL` | Dense front-front queue overlaps remain; honest `FAIL` |

## 5. Verification
- Full unit test suite: `435 tests` (`python -m unittest discover -s tests -q`) — **OK**.
- Visual OSM-vs-3D overlay artifacts saved locally under `G:\3d\gemini\CT-scene-rebuild\runs\gemini-3d-audit-001\`:
  - `overlay-osm-3d-aug26.jpg` vs `overlay-osm-3d-aug26-after.jpg`
  - `overlay-osm-3d-junction-after.jpg`
  - `overlay-osm-3d-oct22.jpg`, `overlay-osm-3d-aug28.jpg`, `overlay-osm-3d-oct26-after.jpg`
