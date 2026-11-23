^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
