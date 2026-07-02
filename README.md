# WAUV Simulation Setup

## Cloning the Repo

Clone this repo in a folder called `WAUV`, preferably at root, so `~/WAUV/<this repo>`

## Installation

Then go to the repo and run `./install.sh` and it will install: **Stonefish**, **ArduPilot**, **MAVROS**, **ROS2 Humble**, and it will build everything that needs to be built. Expect this to take a while (20 ish min?)

## Package Details

### Stonefish_bluerov2

The `Stonefish_bluerov2` package in our `src/` is cloned from [bvibhav/stonefish_bluerov2](https://github.com/bvibhav/stonefish_bluerov2) and it has been modified to work with our setup.

The important files in this which u will probably have to mess with if required are:

- `scripts/ardusim_patch.py` — this file does the communication between ardusub and the sim using UDP JSON packets
- `scenarios/bluerov2_tank.scn` — this file defines the world environment and calls `bluerov2.scn`
- `scenarios/bluerov2.scn` — this fiel defines the sub and its sensors and thrusters

### Stonefish_ros2

The `Stonefish_ros2` package was directly cloned from the stonefish official docs. Since this was originally built to run on ROS2 **JAZZY**, some code was modified to make it run on ROS2 **HUMBLE**.

More details on this: [WAUV Stonefish Setup Doc](https://docs.google.com/document/d/1tLnreV7rijCW11Q3Rp556eHpF6nkKugQAys87H5s26I/edit?usp=sharing)

## Running the Sim

Alternatively you can use the launch file with:

```bash
ros2 launch wauv_sim bluerov2_launch.py
```

Runn these 3 in seperate terminals:

```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:="udp://127.0.0.1:14551@" -p use_sim_time:=false
```

```bash
sim_vehicle.py -v ArduSub -w -L RATBeach --map --console -f vectored_6dof --model JSON --out=udp:127.0.0.1:14551
```

```bash
ros2 launch stonefish_bluerov2 bluerov2_sim.py
```

## Vehicle Control

**To set the vehicle to guided mode:**

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
```

**To arm the vehicle:**

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: True}"
```

**To movevehicle using velocities** (this specific argos will move it 0.5m/s forward and the rate of the message being published is set to 10/s):

```bash
ros2 topic pub --rate 10 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: -0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

```bash
ros2 topic pub --once /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: -1.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

> ^^^these commands work in world frame and not local frame
