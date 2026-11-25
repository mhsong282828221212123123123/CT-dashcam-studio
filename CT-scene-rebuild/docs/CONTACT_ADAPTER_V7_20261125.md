# Contact Adapter v7 (328 Records / 32 Dates) & 3D Pipeline Trade-off Evaluation (2026-11-25)

## 1. Dataset Expansion (`configs/contact-learning-annotations-v7.json`)
- Added **37 visually reviewed unique vehicle crops** extracted via CPU `yolov8n.onnx` from **6 brand-new `E:\tesla` dates** (`2026-08-18`, `2026-08-22` rainy, `2026-09-05`, `2026-10-15`, `2026-10-27` night, `2026-11-10` night) into the `train` split only.
- Total: **328 records across 32 distinct drive dates/sessions** (`train=238`, `validation=64`, `test=26`).
- The 6-session `validation` split (`n=33` in-range) and the 3-date untouched `E:\tesla` `test` split (`sep22`, `oct28`, `nov04`, `n=25`) remain **100% frozen** from `v6` for strict apples-to-apples comparison.

## 2. Single-Frame 3D Ground Projection Comparison (CPU-only)

| Model | Validation 3D Mean / Med / P90 (`n=33`) | Untouched `E:\tesla` Test 3D Mean / Med / P90 (`n=25`) | CPU ONNX Max Diff |
| --- | --- | --- | --- |
| Baseline (YOLO bottom-center) | `0.945 m` / `0.754 m` / `1.718 m` | `0.627 m` / `0.444 m` / `0.822 m` | — |
| `contact-adapter-v6` (291 rec / 26 dates) | **`0.714 m`** / **`0.439 m`** / **`1.246 m`** | `0.518 m` / `0.340 m` / `1.167 m` | `1.01e-6` |
| `contact-adapter-v7` (328 rec / 32 dates) | `0.790 m` / `0.473 m` / `1.339 m` | **`0.422 m`** (-32.6%) / **`0.220 m`** (-50.4%) / **`1.139 m`** | **`2.94e-7`** |

## 3. End-to-End 60s 3D Scene Comparison (`v6` vs `v7`, latest ray-depth `surrounding.py`)

| Scene | Baseline (no adapter) | Opt-in `v6` | Opt-in `v7` | Notes |
| --- | --- | --- | --- | --- |
| `highway` (`curb-steering-highway-002`) | `91 -> 22` (`FAIL`) | **`1 -> 0` (`PASS`)** | `78 -> 2` (`FAIL`, 1 interval) | `v6` achieves 0 overlaps on highway |
| `junction` (`curb-steering-junction-001`) | `5 -> 4` (`FAIL`) | **`5 -> 0` (`PASS`)** | **`5 -> 0` (`PASS`)** | Both pass cleanly |
| `oct26` (`blindspot-junction-oct26-004`) | `137 -> 137` (`FAIL`) | **`138 -> 4`** (`2` fails) | `120 -> 11` (`3` fails) | `v7` had lower raw overlaps (`120` vs `138`), `v6` had lower final (`4` vs `11`) |
| `aug28` (`blindspot-junction-aug28-004`) | `30 -> 16` (`FAIL`) | `42 -> 20` (`4` fails) | `43 -> 20` (`4` fails) | Equivalent |

## 4. Honest Verdict
- Both `models/contact-adapter-v6` and `models/contact-adapter-v7` remain **opt-in / unadopted by default** (`runtime_adopted=false`, `contact_adapter=None`).
- `v7` substantially improves single-frame 3D ground projection accuracy on the untouched 3-date `E:\tesla` test set (`0.627 m -> 0.422 m`, median `-50.4%`), while `v6` remains stronger on end-to-end 60s `highway` (`0` vs `2` frames) and `oct26` (`4` vs `11` frames) final overlap counts. Both models are preserved cleanly for reproducibility.
