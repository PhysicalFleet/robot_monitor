^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.1 (2026-11-23)
------------------
* ``run.py`` at the repository root: run the dashboard, the demo, ``check`` and ``top``
  straight from a git clone, without building.
* Works on a minimal ``ros-core`` install: the TF tab is skipped when ``tf2_msgs`` is missing
  instead of failing at startup.
* Several Robot Monitors on one network no longer re-report each other's findings from
  ``/diagnostics``.
* No issues for hidden helper nodes or DDS endpoints that ROS can't name.
* Use ``logger.warning`` (``warn`` no longer exists in newer rclpy).
* Readable names for ``python -c`` / ``python -m`` processes; unique node names for CLI runs.
* ``check``: a mistyped topic and a /diagnostics WARN are now reported within a short run (the
  waits that keep start-up noise off the dashboard are scaled to the check's duration).

0.2.0 (2026-11-23)
------------------
* TF tab: the live TF tree from /tf and /tf_static with per-frame rates and last update;
  frame names are read from the raw messages, so busy /tf streams stay cheap.
* New rules: a TF frame with two parents, a frame on both /tf and /tf_static, a transform
  that stopped updating, a TF tree split into separate trees.
* Topic panel: "Show the latest message" (cached from the monitor's own subscription when
  possible; long arrays and strings are shortened).
* Node panel: read-only parameters on request, plus the node's services and actions.

0.1.0 (2026-11-23)
------------------
* First release.
* Live web dashboard: overview with health score and issues, nodes, topics, graph, logs,
  diagnostics, shareable Markdown report, baseline of expected nodes and topic rates.
* Node to process mapping on Fast DDS, Cyclone DDS and Zenoh (GUID calibration, remaps,
  composition containers, log files, executable names, shared participants).
* Topic rate, bandwidth, size and header latency from raw messages; sampling of heavy topics
  and a total message budget.
* Health rules: QoS mismatches, mistyped topics, type mismatches, duplicate node names, stale
  and slow topics, CPU saturation, memory growth, restarts, clock skew, under-voltage,
  temperature, memory, disk, /diagnostics and /rosout errors.
* ``ros2 monitor check`` (CI, exit codes, text/Markdown/JSON), ``ros2 monitor top``,
  Prometheus ``/metrics``, findings on ``/diagnostics``.
* Demo robot with typical bugs: ``ros2 launch robot_monitor demo.launch.py``.
