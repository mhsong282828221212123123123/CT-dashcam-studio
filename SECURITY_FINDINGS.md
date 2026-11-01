# Desktop QR sharing findings — owner review required

Date: 2026-11-01. Source baseline af986a8; existing behavior retained. Findings concern local-network access; no remote compromise or Internet exposure was demonstrated. Severity assumes the computer is on a network with untrusted reachable peers. Reproduction uses synthetic test files, not user footage.

## QR-01 — unauthenticated cleartext access (High on an untrusted LAN)

Evidence: network/server.py binds 0.0.0.0, tries 8765 through 8814 and accepts GET /, /index.html, /download, /video.mp4 without credential checks. The URL's t parameter is only a modification timestamp. The server does not enforce TLS. Any reachable peer can retrieve the current exported video and filename; opening a QR link is not an authorization check. Firewalls/routing can limit reachability, but application-level access control is absent.

Reproduce on a trusted test network: export a synthetic clip, open QR sharing, read the displayed host/port, then GET http://HOST:PORT/download from another reachable client without authentication. Expected current result: 200 with the clip bytes. Byte-range /video.mp4 requests are also accepted. Local loopback handler tests reproduce the missing credential requirement without exposing user files. Mitigations such as token authentication or interface changes require a separate approved change; none is implemented here.

## QR-02 — dialog dismissal does not reliably end sharing (High)

Evidence: ui/dialogs.py QRShareDialog.closeEvent calls server.stop, but its button connects to accept. QDialog accept/reject hide the dialog without closeEvent in PyQt6 6.11.0 / Qt 6.11.1. Verified with real Qt widgets and an instrumented server: accept = 0 stop calls, reject = 0, window close = 1. LocalVideoShareServer starts a daemon serve_forever thread; a hidden/destroyed dialog does not itself request shutdown.

Reproduce with a synthetic export: open sharing, note URL, dismiss with the Cancel/confirm button or Esc, then request /download at that address while the app remains open. The current server can remain reachable. Title-bar X stops the currently open dialog's server; it does not retroactively close already leaked servers. Process exit ends daemon threads. A separate lifecycle fix would be appropriate but is explicitly outside this documentation task.

## QR-03 — file state shared across server instances (Medium; compounds QR-02)

Evidence: _start_server assigns VideoShareHTTPHandler.video_path/video_filename on the shared handler class. New server instances overwrite those fields for older still-running instances. An old known URL can serve a subsequently shared export instead of the file originally associated with it.

Reproduce using two tiny synthetic files and two local server instances: start A, request A's /download; start B, request the same A endpoint again. The handler now reads B. Tests isolate the sockets on loopback; production bindings are unchanged. Session-specific handler state and shutdown/access controls need a separate owner-approved implementation.

## Scope

No exploit against arbitrary filesystem paths was found in the fixed download routes. No arbitrary remote code execution claim is made. Network reachability, browser security controls, existing firewall rules and TLS interception were not exhaustively tested. No production network binding, server route, UI share-lifecycle or media code is changed by this task.
