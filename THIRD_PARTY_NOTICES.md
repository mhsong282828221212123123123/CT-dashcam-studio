# CT Dashcam Studio Desktop — third-party notices

Reviewed 2026-11-01. Original project code is offered under **GPL-3.0-only** to the extent the owner can grant those rights. Copyright (c) 2026 Companion Turret and respective contributors. No warranty is provided. Third-party code, fonts, models, artwork, codecs and system components retain their own terms. This notice does not certify a complete redistributable package; see LICENSE_AUDIT.md and REDISTRIBUTION_REPORT.md.

## Original project and historical permissions

LICENSE contains the official unmodified GNU GPLv3 text. The version choice for original project code is GPL-3.0-only, not “or later”. The prior MIT notice is preserved verbatim at licenses/project-before-gpl-MIT.txt. Previously distributed MIT source versions retain their MIT permissions; this migration does not revoke them or retroactively relicense every binary dependency. Existing third-party copyright notices are retained. AI-generated/AI-edited source is not presumed free of third-party obligations.

## Runtime components

| Component | Inspected version | License / retained evidence |
| --- | --- | --- |
| PyQt6 | 6.11.0 | GPL-3.0-only declaration; Riverbank copyright/license in licenses/packages/pyqt6/. PyQt is not LGPL. No commercial license was verified. |
| Qt runtime and plugins | 6.11.1 | PyQt6-Qt6 wheel declares LGPL v3. Qt Core/Gui/Widgets/Network/Svg/Pdf and image/style/TLS/platform plugins are present in the payload. Preserve wheel license in licenses/packages/pyqt6-qt6/ and licenses/Qt-wheel-LICENSE.txt; module and bundled third-party/source requirements need separate verification. |
| PyQt6-sip | 13.12.0 | BSD-2-Clause, retained package license. |
| OpenCV Python | wheel 5.0.0.93 / cv2 5.0.0 | Apache-2.0 plus wheel's LICENSE-3RD-PARTY.txt. Both original notice files retained under licenses/packages/opencv-python/. Includes a separate FFmpeg backend DLL, not the same binary as imageio FFmpeg. |
| NumPy | 2.2.6 | BSD and bundled OpenBLAS/LAPACK/GCC-runtime notices; full installed wheel license retained under licenses/packages/numpy/. Preserve the GCC Runtime Library Exception and all embedded conditions. |
| Pillow | 12.3.0 | MIT-CMU and embedded library notices in its supplied LICENSE, retained under licenses/packages/pillow/. |
| qrcode | 8.2 | BSD, full original license in licenses/packages/qrcode/. |
| colorama | 0.4.6 | qrcode Windows transitive dependency, BSD declaration; supplied license retained under licenses/packages/colorama/. |
| imageio-ffmpeg | 0.6.0 | BSD-2-Clause for the Python wrapper, retained under licenses/packages/imageio-ffmpeg/. This does not license its FFmpeg executable under BSD. |

The dated, exact active dependency graph (including markers/inactive extras), copied-notice paths and hashes is in DEPENDENCY_AUDIT.json. Optional development/documentation/test extras from package metadata are not assumed to be runtime dependencies.

## FFmpeg and native code

The imageio executable is **ffmpeg-win-x86_64-v7.1.exe**, Gyan 7.1 essentials, built with GCC 14.2.0. Its flags include --enable-gpl, --enable-version3, --enable-static, libx264, libx265 and many other libraries. Its own -L output states GPLv3-or-later. It is not an LGPL-only binary. The full configuration, hash and license output are retained in DEPENDENCY_AUDIT.json. Application GPL-3.0-only does not narrow the executable's upstream “or later” grant. Preserve applicable notices and provide complete corresponding source/build materials for this exact binary before redistribution; a bare latest-source link is not proof of compliance.

OpenCV's opencv_videoio_ffmpeg500_64.dll contains an LGPL-2.1-or-later libswscale statement and a different static configuration using libopenh264/libvpx/libaom. The full OpenCV wheel third-party notice covers its supplied notices, but exact source/build coverage remains to be assembled. Its presence must not be confused with the GPL imageio executable.

The retained payload also includes Python 3.10.6 runtime/extensions, OpenSSL 1.1.1n DLLs, libffi, compression libraries, NumPy/OpenBLAS native code, Qt/PDFium/image codecs and MSVC 14.x runtime DLLs. BINARY_AUDIT.json lists all 75 native files with SHA-256, PE import names and file versions where present. These components are not all relicensed GPL. Python's supplied LICENSE is retained when available; platform-library distribution rights and complete source/notice coverage are unresolved, especially Qt's embedded third-party components. See the audit rather than inferring permission from DLL filenames.

## Models, fonts and artwork

- **YuNet face model:** models/face_detection_yunet.onnx, SHA-256 8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4. Matches the official OpenCV Zoo face_detection_yunet_2023mar.onnx LFS pointer (232,589 bytes). MIT, copyright (c) 2020 Shiqi Yu; full text licenses/YuNet-MIT.txt and source/hash pointer licenses/YuNet-model-lfs.txt retained. Source: https://github.com/opencv/opencv_zoo/tree/main/models/face_detection_yunet .
- **YOLOv8n:** models/yolov8n.onnx, SHA-256 d1062b1827f1e8f2319cfcee5a55278d477d0d875d14628ead9686d276cc61b8. Embedded author Ultralytics; exporter 8.4.139; embedded license AGPL-3.0. Model terms remain AGPL, not original-project GPL-only. AGPL text is preserved in licenses/AGPL-3.0.txt and the Nuitka supplier copy, but a model-specific source/training/export chain and combined-work treatment remain unresolved. No verified enterprise grant is present. https://www.ultralytics.com/license .
- **Pretendard Regular/SemiBold/Bold:** SIL Open Font License 1.1; retain assets/fonts/OFL.txt, README.md and licenses/Pretendard-OFL.txt. Reserved font names and copyright statements remain intact. Original hashes/source are in assets/fonts/README.md. Fonts are loaded privately, without system installation or runtime download. https://github.com/orioncactus/pretendard .
- **icon.ico, steering_hud.png, screenshots:** preserve existing files. Supplied artwork/Portal-inspired branding and any visible Tesla/third-party marks have not been independently cleared. No Valve or Tesla rights are granted by the project license. Full file hashes are in the asset section of DEPENDENCY_AUDIT.json.
- **OSM map data:** © OpenStreetMap contributors, ODbL 1.0. Tile-provider terms are separate from application licensing. Existing on-video attribution/prefetch/cache policy gaps remain documented. https://www.openstreetmap.org/copyright and https://operations.osmfoundation.org/policies/tiles/ .

## Build tools and emitted runtime

Nuitka **4.1.3** declares AGPLv3 for the compiler, with a separate **Nuitka Runtime Library Exception 1.0** permitting independent compiled output under other terms. Both the exact supplied LICENSE.txt and LICENSE-RUNTIME.txt plus NOTICE.txt and included vendor notices are preserved under licenses/packages/nuitka/. Do not mislabel this installed Nuitka as Apache-only or omit its runtime exception when auditing output. Inspect marked emitted runtime files when redistributing; the exception does not relicense the compiler or other AGPL components such as the YOLO model.

ordered-set 4.1.0 is a build-only package with an MIT classifier but no separate notice file found in this installation; source notice verification remains open if redistributing that tool. zstandard 0.25.0 is BSD-3-Clause with supplied notice retained; it supports onefile compression and may correspond to emitted/vendored native runtime. MinGW GCC 15.2.0 was used for the inspected Nuitka build. Tool/source/runtime-exception obligations must be checked before packaging compilers or their runtime libraries. None of these tools is automatically a separately shipped Python application module.

## Legal documents in portable builds

The updated build stages LICENSE, this notice, PRIVACY.md, audits, security findings, original license files and build-info.json under legal/ and includes them in standalone and onefile resources. The Privacy & Licenses viewer reads those files offline. Adding them does not supply missing exact dependency sources or resolve the AGPL-model/QR/OSM findings. Do not publish a new release until the documented redistribution blockers have been resolved and reviewed.
