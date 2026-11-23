# Contributing to Robot Monitor

Thanks for helping! Bug reports with a `ros2 monitor check --format md` report attached are the
easiest to act on.

## Development

```bash
mkdir -p ~/ns_ws/src && cd ~/ns_ws/src
git clone https://github.com/PhysicalFleet/robot_monitor.git
cd .. && rosdep install --from-paths src -y --ignore-src
colcon build --packages-select robot_monitor
colcon test --packages-select robot_monitor && colcon test-result --verbose
```

The dashboard is plain HTML/CSS/JavaScript in `robot_monitor/robot_monitor/web/` with no build step;
rebuild the package (or run `colcon build --symlink-install`) and reload the page.

## Principles

- **Zero configuration.** Everything works out of the box; parameters only turn things off.
- **Never hurt the robot.** Bounded CPU and memory, read-only, no changes to the system.
- **No false alarms.** A rule that fires wrongly costs more than a missed nit: rules must be
  conservative and come with tests.
- **Plain words.** Every issue says what is wrong, why it matters and how to fix it.
- **Standard library only** at runtime (beyond ROS packages), so it installs everywhere.

## Code style

`ament_flake8`, `ament_pep257` and `ament_copyright` run as part of `colcon test`.
