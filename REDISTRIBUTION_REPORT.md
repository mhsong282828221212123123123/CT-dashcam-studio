# Desktop redistribution report

Date: 2026-11-01. The initial audit authorized local changes only. The owner subsequently explicitly requested source push and registration of the latest release as stable with updated licensing. That publication authorization does not clear the third-party/source/security findings below or constitute legal certification.

## Existing published archive

The already available local CT_Dashcam_Studio_v1.3.1_Portable.zip was inspected; it was not redownloaded from GitHub or modified. SHA-256: e65f70788e33aff407edaf5411a67fb475bce19a7c87090b23f2b51f8cdcf54c. Files: CT_Dashcam_Studio.exe, MIT LICENSE.txt, OFL.txt. It does not include comprehensive third-party notices/privacy/audit materials or an exact-source bundle/offer for its GPL components. Historical MIT grants for original code remain; dependency rights were always separate.

## Newly prepared packaging

- tools/prepare_legal_bundle.py stages LICENSE, PRIVACY.md, THIRD_PARTY_NOTICES.md, LICENSE_AUDIT.md, this report, security/discovery/test reports, dependency/native inventories and licenses/ unchanged originals under legal/.
- tools/build_windows.ps1 includes that whole legal directory in standalone and onefile builds; retains the preexisting root LICENSE/icon/assets/models. Version remains APP_VERSION 1.3.1. A file-description field identifies original-code GPL-3.0-only without purporting to license third parties.
- legal/build-info.json records application version, SPDX scope, source revision, dirty-working-tree status, platform/tool version information, and source/legal file hashes. A source tree digest identifies an uncommitted local build honestly rather than pretending the old commit includes new files.
- The legal viewer reads the bundled files offline. Standalone folders and onefile extraction both keep relative legal paths. A portable-legal companion directory is staged beside the build output so a future ZIP can include readable notices without executing the app. Public upload occurs only under the subsequent explicit owner instruction.
- Exact package versions are in requirements-audited.txt and DEPENDENCY_AUDIT.json; wheel hashes and full upstream source/build chain are still needed for reproducible binary redistribution.

## Outstanding release blockers / owner review

1. Resolve the AGPL-tagged YOLO model's rights, preferred source/training/export provenance and combined-work obligations. Do not advertise the whole binary/model bundle as exclusively GPL-3.0-only.
2. Provide complete corresponding source and build materials (or a legally sufficient applicable distribution method) for the exact GPL FFmpeg binary and its static dependencies, plus PyQt/Qt and applicable native components. A current upstream homepage or a promise to upload later is not the completed obligation. No unsupported written-source offer is invented here.
3. Complete Qt module/plugin/embedded-third-party notices and LGPL modification/replacement/relinking requirements for the selected packaging mode; review all native DLL rights. Maintain Python, OpenCV/Pillow/NumPy and tool runtime-exception notices.
4. Identify creators/licenses for icons, steering image and any copied source not proven original. Preserve pre-migration MIT and all third-party notices.
5. Review OSM prefetch/cache/attribution policy conflicts and the separately verified QR sharing weaknesses. Functional fixes require another approved scope; this task does not disable or redesign them.
6. Before a future release, preserve an exact public source revision, build instructions, corresponding dependency sources and checksums alongside the artifact. Re-run audit/build checks after any dependency or model change.

## Evidence limits

This review inspected the actual source, installed build environment, retained native payload, model metadata and a local copy of the existing archive. It does not prove ownership of every upstream file, certify all patents/territories or establish a valid source-distribution arrangement for every library. No application functionality was changed to conceal a licensing issue. See COMPLIANCE_VERIFICATION.md for performed tests and physical-device/real-footage limits.
