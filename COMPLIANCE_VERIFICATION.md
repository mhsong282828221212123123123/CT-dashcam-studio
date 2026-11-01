# Desktop compliance verification — 2026-11-01

Baseline: af986a8 / version 1.3.1. Documentation/legal UI and packaging only. This report distinguishes performed checks from release clearance. This describes the initial audit. The owner subsequently explicitly requested source push and updated stable release registration; that publication is a separate authorized action and does not resolve the audit findings.

## Discovery / evidence

- Inspected source entry, five-language i18n, packaging script, privacy/image/telemetry/network code, visible copyright history and prior MIT license.
- Inspected the local existing portable ZIP (hash matches the previously uploaded v1.3.1 ZIP); did not download it again. Found only executable, MIT LICENSE.txt and OFL.txt. Scanned 75 native files in its retained standalone build payload and 12 actual runtime/build packages. Full supplier notices retained.
- Read ONNX metadata without adding an ONNX dependency. YOLO declares AGPL-3.0; YuNet hash matches the official OpenCV Zoo pointer and its MIT license. Preserved Pretendard/OFL and historical MIT.
- Reproduced with real PyQt widgets/instrumented test server: accept/reject hide QR dialog without server.stop; close invokes it once. Security tests use synthetic files/loopback only; no real footage is exposed.

## Validation results

- `python -m unittest discover -s tests -v`: **49 tests passed** (40 existing plus 6 legal/resource tests and 3 QR-finding reproductions), 69.086 seconds. No original footage or external update requests used.
- Source `python CT_Dashcam_Studio.py --build-check ...`: **ok: true**. Verified Pretendard, all ten themes, camera popup, export blur/progress cleanup, three offline legal tabs, GPL/notice resources, both local ONNX models loading, real synthetic FFmpeg encode/OpenCV decode, clip-loader and export QThread signals. Detailed JSON is kept with local test artifacts, not an external telemetry upload.
- Native onefile build uses `tools/build_windows.ps1 -Mode onefile -OutputDir build/nuitka-legal`. The internal onefile build completed successfully (GCC 15.2.0), with 49 legal data files included. Its compiled `--build-check` exited 0 with **ok: true**, including three offline legal tabs and the same model/font/encode/decode/thread checks as the source run. Executable PE ProductVersion/FileVersion are 1.3.1.0 and FileDescription explicitly states GPL-3.0-only original code with separate third-party terms. Detailed local JSON: `outputs/desktop-legal-native-check.json` in the workspace artifact directory. A successful internal build is not authorization or clearance for public redistribution.
- No configured lint or static-type-check command exists. Changed Python modules are syntax-compiled in validation; existing unit/source/native checks are used instead of claiming a nonexistent lint run. Unit tests and source/native packaging checks use synthetic media and suppress the GitHub update request; they do not establish facial/plate recognition accuracy or full real-Tesla multi-camera synchronization. An internal local build is not a release-clearance certificate.

## Scope invariants

No changes are intended in core/decoder.py, renderer.py, exporter.py, scanner.py, privacy_filter.py, plate_enhancer.py, map_loader.py, utils.py, network/server.py, network/updater.py, ui/dialogs.py, loader_worker.py, widgets.py, export_progress.py, theme/typography implementations, bundled model/font/icon bytes or application entry logic. Main-window changes only add/retranslate a legal footer button and open an isolated dialog. core/constants.py adds APP_LICENSE; i18n adds legal strings; build_check adds legal-resource verification. No analytics, tracking, mandatory consent modal, model download or map/QR gate is added.

## Review limits

No complete real Tesla footage playback/export/SEI/face/plate accuracy check or physical smartphone transfer is claimed. No packet-capture audit of every OS/provider request was performed. External traffic is documented from source and local controlled tests. Source/readable notices do not resolve the AGPL model's combined-work/source requirements, exact FFmpeg/Qt/PyQt/native source/notice obligations, artwork provenance, OSM policy conflicts or QR security findings. See LICENSE_AUDIT.md, REDISTRIBUTION_REPORT.md and SECURITY_FINDINGS.md. No library/model/network behavior was changed to hide those issues.


## Post-build evidence and limits

The existing ZIP executable SHA-256 matches the pre-change retained onefile executable exactly: 77dc87e314926abce1b40a946eac91c71ef6047b7104fd2e732b5a348b446ec1. This connects the archive review to the retained build environment; full upstream source compliance is still unresolved.

A compiled GUI launch exposed the localized Privacy & Licenses footer button through Windows accessibility. Windows screenshot capture failed with a frame timeout, so no pixel-level visual approval or successful manual dialog interaction is claimed. Automated tests loaded all original notice documents in all five languages; the compiled offline check instantiated the same dialog and loaded its bundled resources. Native UI checks used the Computer Use skill; they introduced no application telemetry or functional changes.

The packaged legal snapshot/manifest was prepared before this post-build verification annotation. This final report supplements that internal candidate; re-stage and rebuild all documents after owner review before any future release. No public release, upload, or old-archive change was performed during that initial validation. Subsequent publication is authorized explicitly by the owner and uses a fresh clean-source build. Original media/network modules remain byte-identical to the baseline.

Audit-script-only existing tooling (not imported by the app): packaging 26.3, pefile 2024.8.26. These tools inspect metadata/PE files and do not become application runtime dependencies.
