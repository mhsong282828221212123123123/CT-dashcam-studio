# Windows portable build

Run from the repository root with Python 3.10 x64. The verified toolchain is
Nuitka 4.1.3 with MinGW64 GCC 15.2.0. Nuitka generates C and uses the native
compiler to produce the executable; the output does not require a separate
Python installation.

```powershell
python -m pip install PyQt6 opencv-python numpy Pillow qrcode imageio-ffmpeg
python -m pip install nuitka==4.1.3 ordered-set zstandard
./tools/build_windows.ps1
```

The output is `build/nuitka/CT_Dashcam_Studio.dist/CT_Dashcam_Studio.exe`.
Keep the **whole .dist folder** together when distributing it as a ZIP.
The executable uses adjacent Qt libraries, FFmpeg, fonts, icon, and models.
The script includes the entire `assets` and `models` directories, application
license and icon. The Nuitka imageio_ffmpeg package configuration includes its
FFmpeg executable. The Pretendard OFL license remains in `assets/fonts/OFL.txt`.

A silent offline packaging check is available:

```powershell
$application = (Resolve-Path 'build/nuitka/CT_Dashcam_Studio.dist/CT_Dashcam_Studio.exe').Path
$check = Join-Path $env:TEMP 'ct-packaging-check.json'
$process = Start-Process -FilePath $application -ArgumentList @('--build-check', ('"' + $check + '"')) -WindowStyle Hidden -Wait -PassThru
Get-Content -LiteralPath $check
if ($process.ExitCode -ne 0) { throw 'Packaging check failed' }
```

The check creates no visible app window and skips the update request. It verifies
actual Pretendard font matching, ten themes, resource files, both OpenCV models,
FFmpeg encoding/OpenCV decoding, and application clip-loader/export QThreads and
their completion signals. It also writes two small test MP4s beside the JSON.
It does not validate face/plate recognition accuracy on real dashcam recordings.

## Single executable release

For the same installation style as the existing GitHub releases:

```powershell
./tools/build_windows.ps1 -Mode onefile
```

This creates `build/nuitka/CT_Dashcam_Studio.exe`. It bundles the standalone
runtime and resource files, extracting them privately when launched. Distribute
this executable in the release ZIP together with the font license notice.
The same `--build-check` command can verify the onefile executable.


## Legal resources and release review

The build now stages reviewed documents and all `licenses/` originals through `tools/prepare_legal_bundle.py`, bundles them as `legal/`, and writes `build-info.json` with original-code SPDX `GPL-3.0-only`, version, source revision/dirty status and source/legal hashes. The existing root LICENSE remains included. The footer's Privacy & Licenses dialog reads these files offline in source, standalone and extracted onefile modes. A readable `portable-legal/legal/` companion is also staged next to the output; include it with a future ZIP, rather than requiring users to execute a binary to read its terms.

The old instructions' suggestion to ship only the executable and font notice is superseded: include project, privacy and full third-party legal material, exact matching source/build information, and the required corresponding sources for distributed copyleft components. See `REDISTRIBUTION_REPORT.md` before publishing. No new public release is created by this build script.

Use `requirements-audited.txt` and `requirements-build-audited.txt` to identify the tested installed versions; these are snapshots, not a wheel-hash or corresponding-source lock. Re-run `python tools/audit_distribution.py --archive PATH` when auditing an existing local ZIP. This reads local binaries and copies supplier notices, never downloads a release or changes a model/library.

`yolov8n.onnx` embeds AGPL-3.0; the original-code GPL label does not relicense it. The exact-source/Qt-native notice/asset-provenance and QR/OSM issues remain release-review items. Internal build/check success does not waive those obligations.
