# robot_monitor

Zero-config health monitor for ROS 2 with a live web dashboard: every node mapped to its
process (CPU, memory, restarts), topic rates, bandwidth and latency, QoS mismatches,
mistyped topics, duplicate node names, stalled sensors, memory leaks, under-voltage, and a
fix for each problem.

```bash
ros2 run robot_monitor robot_monitor          # dashboard on http://<robot>:9191
ros2 launch robot_monitor demo.launch.py  # try it on a demo robot full of bugs
ros2 monitor check                  # one-shot report for the terminal or CI
ros2 monitor top                    # htop for ROS nodes
```

Full documentation: https://github.com/PhysicalFleet/robot_monitor
