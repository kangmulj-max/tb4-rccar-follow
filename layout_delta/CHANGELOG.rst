^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package layout_delta
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-10-05)
------------------
* scan_mapper: round occupancy grid on a fixed reference map, stop-and-observe integration, TF fallback, save service, save on shutdown
* patrol: localization gate, Nav2 waypoint patrol with retry/skip, mission log
* detect: round-to-round diff (observed-in-both rule, wall-jitter margin, moved pairing), report (md/png/json), ground-truth evaluation
* scan_mapper: AMCL covariance gate (max_pose_std_m 0.25)
* patrol: a goal timeout is always recorded as failed (stale SUCCEEDED status after cancel)
* detect: shift margin, min extent (single box face), patrol-route ROI, eval.json; moved detections count for removed truth
* unit tests (10), simulation round test on the 2026-10-04 real SLAM-map world
