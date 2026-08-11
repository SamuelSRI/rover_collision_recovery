# rover_collision_recovery 3.0

This node is placed immediately before Nav2 Collision Monitor:

```text
/cmd_vel_smoothed
        |
        v
rover_collision_recovery
        |
        v
/cmd_vel_recovery
        |
        v
Nav2 collision_monitor
        |
        v
/cmd_vel -> motors
```

## Detection strategy

- `/collision_monitor_state` is the primary trigger.
- `/scan` is transformed into `base_link`.
- Before every escape, compact front, rear and side zones are checked.
- While moving, the compact zone is checked continuously.
- Collision Monitor then validates the proposed escape command.
- If either safety layer rejects the direction, another direction is tested.
- If no direction is safe, the rover stays stopped in `WAIT_CLEAR`.

## Install

```bash
cd ~/ros2_ws/src
rm -rf rover_collision_recovery
unzip ~/Downloads/rover_collision_recovery_v2.zip

cd ~/ros2_ws
rm -rf build/rover_collision_recovery install/rover_collision_recovery
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rover_collision_recovery --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch rover_collision_recovery collision_recovery.launch.py
```

## Required Collision Monitor parameters

```yaml
cmd_vel_in_topic: cmd_vel_recovery
cmd_vel_out_topic: cmd_vel
state_topic: collision_monitor_state
```

The static transform `base_link -> lidar_360_link` must be published.


## Recovery release condition

The node no longer uses a clear timer or cooldown timer to return control to
Nav2. It remains in `WAIT_CLEAR` until:

1. `/collision_monitor_state` no longer reports `STOP`;
2. the compact LaserScan zone associated with the original collision is free;
3. the scan is recent.

The movement durations still limit each automatic escape attempt, but they do
not release recovery.
