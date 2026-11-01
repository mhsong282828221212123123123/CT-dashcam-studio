# CT Dashcam Studio Desktop — Privacy Policy

Last updated: 2026-11-01. English is the canonical policy language. This policy describes the Python/Windows source revision containing the new legal-information UI; older releases may not include these documents. It is separate from the Web edition's analytics consent policy.

## Operator and contact

The project is maintained by Companion Turret under GitHub account mhsong282828221212123123123. It is an independent tool, not affiliated with Tesla, Inc. No company, private address or email is invented here. Contact the maintainer through [GitHub Issues](https://github.com/mhsong282828221212123123123/CT-dashcam-studio/issues) or the optional [feedback form](https://forms.gle/Lh1doyRZNAZ6cqRk9). Issues are public; avoid posting private footage, identifiers or GPS information. The Google Form requires Google sign-in for file uploads; anything you deliberately send there leaves your device and is subject to Google's policies.

## Local files, footage, telemetry and AI

The application reads the folders/files you choose. Filenames/timestamps group clips and determine recording times; video frames are decoded locally with OpenCV, synchronized and composed on your device. The SEI parser reads embedded vehicle telemetry, including speed, steering, indicators, pedals, mode and GPS. Rendering and FFmpeg encoding run locally; no application video-upload endpoint or cloud inference request was found.

Face, vehicle/plate-region processing and plate enhancement use local image processing and bundled ONNX models through OpenCV. No runtime model download is implemented. Frames, detection boxes and model buffers are held in process memory. This is not a promise that detection is accurate, nor a claim that the model licenses are the same as the application's; see LICENSE_AUDIT.md.

Export files are written to the location you choose and remain there until you delete/move them. Cancelling export attempts to remove incomplete output; an error/interruption or another process holding a file can leave partial output. There is no verified automatic expiry or secure-erasure policy for exports, originals or recipient copies. Source inspection found no feature that uploads or deletes your original clips. Operating-system caches, page files, backups, security software and external file synchronization are beyond the application's control.

## GPS and external maps

The minimap requests HTTPS raster tiles from tile.openstreetmap.org. Tile x/y/zoom disclose the requested map area, together with ordinary network metadata such as source IP and the application's User-Agent. The desktop implementation also prefetches up to 90 route-related tile coordinates after clip loading, sampled from decoded telemetry. It may do so before you view those areas. Requests do not contain a complete raw telemetry/GPS-trace document, but their sequence can reveal locations or a route. Do not describe this as entirely offline or entirely local GPS processing.

Tiles are retained in a process-wide memory cache capped at 600 entries, not in a durable application tile database. Closing the process ends that memory cache. The current loader does not implement HTTP cache-header retention/revalidation or a seven-day disk cache. OSM provider policy issues, including prefetching and attribution, are documented in LICENSE_AUDIT.md; this documentation task does not modify map behavior. OSM Foundation controls its own logs/retention.

## Updates and other outbound requests

On startup an UpdateCheckWorker requests the GitHub API releases/latest endpoint for this repository, using User-Agent CT-Dashcam-Studio and a five-second timeout. That request reveals normal connection metadata to GitHub; it does not include selected video files, filenames, frames or GPS in application parameters. The returned version is compared locally to APP_VERSION. If you agree to open a newer version's page, the browser navigates to GitHub. The app does not automatically install an update in the inspected flow.

No Google Analytics tag, telemetry/diagnostics upload endpoint, user account system or server-side user profile was found in the desktop source. No tracking is added by this change. The feedback form and source/releases links open external websites only when selected. External browser activity is governed by that browser's cookies/extensions and the provider's policies, not by a desktop analytics-consent record.

## QR wireless transfer and local-network access

Opening the QR transfer dialog starts an HTTP server on all IPv4 network interfaces (0.0.0.0), trying ports 8765–8814. The QR encodes an address such as http://LAN-IP:PORT/?t=MODIFICATION-TIME. The timestamp is a cache-busting value, not a secret or access credential. get_local_ip uses a UDP socket connect to 8.8.8.8:80 to select a route, with hostname resolution as fallback; the helper does not send a UDP payload.

The server has no password, authentication token, TLS or verified per-client authorization. Anyone able to reach the listening interface/port, subject to firewall/routing, can request the sharing page, /download or /video.mp4 (including byte ranges) and obtain the current shared export and its basename/size. It is not a directory-listing or arbitrary-file browsing server in the inspected routes. It is not guaranteed to be limited to trusted Wi-Fi: other reachable interfaces/routes and firewall settings matter. The app does not itself configure router port forwarding.

**Important existing lifetime limitation:** closing the QR window using its title-bar X invokes server.stop. The current Cancel/confirm button uses QDialog.accept, and Escape uses reject; those paths hide the dialog without invoking its closeEvent in the tested PyQt runtime. A server can therefore continue serving after the dialog disappears, until explicitly stopped or the application process exits. Opening another share can also change the shared file on an older server, because handler file fields are class-level mutable state. Do not rely on a hidden dialog as proof that sharing ended. These verified issues are reported separately and intentionally not fixed by this documentation task.

Recipients can save their own copies; ending sharing does not revoke or delete them. The server reads the export in place and does not make a separate server-side copy. No automated expiration/session-token rotation was found. HTTP request logging is suppressed by the handler, though network/OS tools and recipient browsers may retain their own records.

## Preferences, temporary files and diagnostics

Qt QSettings persists app_language and app_theme under CompanionTurret/CT_Dashcam_Studio (normally Windows per-user registry settings). They have no application expiry. The inspected settings do not persist footage or a complete telemetry history. QR copying puts the share URL on the OS clipboard; clipboard history/synchronization is governed by your OS settings.

The app uses QTemporaryDir for generated theme-control icons. Nuitka onefile builds extract bundled runtime/resources to a temporary directory; cleanup is managed by the packager/OS and can be interrupted. These extracted resources are not a deliberate cache of selected dashcam recordings. Build/test checks can write synthetic MP4s and JSON reports to a requested location; these are diagnostic/developer actions.

Python stdout/tracebacks and captured FFmpeg error output are used for error diagnosis. No application-managed persistent log file, upload or rotation/retention period was found. Source launches, shell redirection and developer build/test tools may save output; it can include local file paths/errors. The windowed portable executable normally has no console, which is not a guarantee that the OS or another tool records nothing. No fixed personal-data retention period is claimed.

## Third-party services and limits

- [GitHub Privacy Statement](https://docs.github.com/en/site-policy/privacy-policies/github-general-privacy-statement)
- [OpenStreetMap Foundation Privacy Policy](https://osmfoundation.org/wiki/Privacy_Policy)
- [OpenStreetMap copyright](https://www.openstreetmap.org/copyright)
- [OSM tile usage policy](https://operations.osmfoundation.org/policies/tiles/)
- [Google Privacy Policy](https://policies.google.com/privacy)

The project operator does not control provider logs, external browsers, downloaded copies or operating-system retention. Provider/jurisdiction-specific retention and legal obligations have not all been verified. This policy describes inspected processing; it is not a claim of complete privacy/security or redistribution compliance. The legal-information viewer opens bundled documents offline and contacts no external service unless you select an external link.
