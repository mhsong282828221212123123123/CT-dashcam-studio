# Camera presence versus estimated position

The earlier renderer made every non-`observed` position translucent. Consequently, continuously detected vehicles became ghosts merely because their depth used held pitch or image-size continuation. That conflated two independent facts.

`Actor.image_observed` now records current synchronized primary-camera presence. The authoritative per-frame source set excludes rear-only support, tentative/association-only detections and central-boundary-excluded detections. World-track source history also identifies a currently visible vehicle when only a predicted metric pose survives and its output source tuple is empty. Presence is recomputed after offline recovery; old anchor flags cannot turn a genuinely blind interval opaque.

The renderer uses opaque meshes/shadows for camera-confirmed vehicles while keeping `pitch_held`, `image_supported` or other estimated position states intact. HUD text separately counts visible vehicles with estimated positions. An unobserved predicted/interpolated vehicle still uses the fading ghost model. Lane voting and motion-heading evidence still depend on position state, not opacity or this presence flag.

This is a display/presence-policy correction, **not an improvement to metric position accuracy**. The measured maximum coordinate change against the preceding continuity repair is0m for the full reviewed source. Raw observations, camera geometry, track IDs and source videos are unchanged.

Artifacts: `../scene-results/visible-presence-review/`. Final output is `camera-confirmed-vehicles-4camera-35s.mp4`; earlier `visible-vehicles-solid-4camera-35s.mp4` is superseded. `final-audit.json`, `final-code-hashes.json` and the MP4 report preserve evidence.

Source review:24/26/30s front vehicles are camera-confirmed and opaque despite estimated depth. At27.4s the wiper causes a real detection dropout: the affected vehicle remains a ghost with the same identity. Tests exercise exact opaque-body rendering, a genuine camera dropout, and a metric-only prediction whose identity is nevertheless still camera-visible. Existing ghost fading/expiry behavior remains tested.

Existing absolute-position, missed-new-object, junction/ramp and unmarked-median limitations remain. Full scene quality is not accepted merely because vehicles are opaque.

Final273tests passed90.053s; diff check passed. Final MP4 decoded1050frames/30fps/960x820/35s, and exported26/27.4/30s images were visually inspected against the four originals. No build/release/push was performed.
