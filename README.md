# Autolivery-Drone

Autonomous parcel delivery for PX4 drones and planes, on ROS 2.

A delivery is one action goal: the vehicle plans a route through the city (A* or RRT*), takes off, flies it in PX4 offboard mode through MAVROS, dodges other traffic on the way, drops the parcel (hover, winch, landing or air drop), flies home and lands. Everything is simulated in a Gazebo city generated from the same map the planner flies against.

## Architecture

```
Deliver goal -> mission_node -> Mission (autolivery_core) -> setpoints (20 Hz) -> MAVROS -> PX4
                    ^               |  plan:  A* / RRT* over the city map, shortcut smoothing
                    |               |  fly:   carrot path follower + velocity-obstacle avoidance
                    |               |  drop:  hover / winch / land / air drop (ballistic lead)
                    |               |  guard: takeover, disarm, battery, timeouts, geofence
                    |               +-> payload_node -> MAVLink gripper cmd | Gazebo DetachableJoint
   MAVROS state, pose, battery, extended state, payload state, /autolivery/obstacles
                                                   ^                ^
                                        adsb_traffic_node     sim_traffic (Gazebo)
```

The decision-making lives in `autolivery_core`, a plain C++17 library with no ROS in it. The nodes only translate, so the whole delivery (takeoff to touchdown, including every failure path) is unit tested against a kinematic PX4 stand-in in milliseconds.

## Repository layout

```
ros2_ws/src/
  autolivery_core/        C++ library: world map, voxel grid, A*, RRT*, smoothing, path
                          follower, velocity-obstacle avoidance, vehicle profiles, gripper
                          commands, the Mission state machine, KinematicVehicle (fake PX4)
  autolivery_interfaces/  Deliver action, PlanPath service, MissionStatus / Obstacle / Payload msgs
  autolivery_drone/       ROS 2 nodes, launch files, vehicle + gripper configs, city map
    src/  mission_node, payload_node, planner_node, adsb_traffic_node, fake_fcu_node
    config/vehicles/      quadcopter, hexacopter, octocopter, coaxial_octocopter,
                          helicopter, vtol, fixed_wing
    config/grippers/      simulated, mavlink_gripper, servo, electromagnet, winch
    maps/urban_delivery.yaml   the city: geofence, buildings, trees, masts, no-fly zones,
                               depots, named dropoffs, simulated traffic
  autolivery_gazebo/      Gazebo world (generated from the map), PX4 vehicle + parcel models,
                          sim_traffic_node, sim.launch.py
.github/workflows/ci.yml  build + all tests in ros:jazzy
```

## Vehicles and grippers

| Profile | Class | Dropoff | PX4 airframe | Gazebo |
|---|---|---|---|---|
| `quadcopter` | multicopter | hover, 3 m | 4001 (x500) | yes |
| `hexacopter` | multicopter | winch, 12 m | 6001 | - |
| `octocopter` | multicopter | land | 8001 | - |
| `coaxial_octocopter` | multicopter | hover, 4 m | 12001 | - |
| `helicopter` | multicopter | winch, 15 m | 16001 | - |
| `vtol` | vtol | hover, 4 m (FW cruise on long legs) | 4004 (standard_vtol) | yes |
| `fixed_wing` | fixed-wing | air drop, 30 m, from the airstrip | 4003 (rc_cessna) | yes |

Any airframe PX4 flies works: pick the closest profile and tune its numbers. Profiles are validated at startup (a fixed-wing with a hover dropoff, a clearance smaller than the airframe, etc. refuse to start).

| Gripper | How it is driven | Release confirmed by |
|---|---|---|
| `simulated` | Gazebo DetachableJoint via ros_gz_bridge | the joint's own state |
| `mavlink_gripper` | `MAV_CMD_DO_GRIPPER` (PX4 payload deliverer) | timing (or a sensor) |
| `servo` | `MAV_CMD_DO_SET_ACTUATOR` on "Peripheral via Actuator Set N" | timing (or a sensor) |
| `electromagnet` | `MAV_CMD_DO_SET_ACTUATOR`, energised to hold | timing (or a reed switch) |
| `winch` | `MAV_CMD_DO_WINCH` deliver / lower, then retract + lock | timing incl. pay-out |

A payload sensor (limit switch, load cell) on `payload_sensor_topic` (`std_msgs/Bool`) turns on sensor-based confirmation and retries (`mission.payload_feedback: true`).

## Setup

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, PX4 v1.16.

```bash
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-ros-gz
# MAVROS needs the geoid dataset, or it aborts at startup:
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

cd ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

For simulation, also build PX4 SITL once:

```bash
git clone --recursive --branch v1.16.2 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot && make px4_sitl
export PX4_DIR=~/PX4-Autopilot
```

## Build and test

```bash
cd ros2_ws
colcon build
colcon test && colcon test-result --verbose
```

What the tests cover:

- **autolivery_core**: map geometry (exact segment/box/cylinder tests), voxel inflation, A* and RRT* (optimality, determinism, time budget, failure messages), smoothing, path following, avoidance (simulated head-on, crossing, descending and hovering encounters keep separation), profile validation, gripper MAVLink mapping, and full mission scenarios for every vehicle class and dropoff method, plus: low and critical battery, pilot takeover, autopilot failsafe, disarm in flight, stuck gripper, cancel in flight and on the ground, vehicle never ready, head-on traffic.
- **autolivery_drone**: the shipped city map loads, every vehicle profile validates, the quadcopter can deliver to every named location, the runway approach is flyable, message and geodetic conversions; every shipped profile flies a whole delivery in the shipped city against the fake vehicle (catches a profile that plans but cannot be flown, e.g. a turn radius that never closes on a waypoint); and launch tests that run the real nodes against `fake_fcu`: a whole quadcopter delivery and a whole fixed-wing air drop (including the runway landing-pattern upload) through the `Deliver` action, plus refusals (unknown location, no-fly zone, inside a building, already busy) and an operator cancel.
- **autolivery_gazebo**: the committed world matches the city map exactly (every collision where the planner thinks it is), PX4 sensor systems are loaded, parcel models fit each airframe.
- **Style**: `ament_uncrustify`, `ament_cpplint` and `ament_flake8` run as part of `colcon test`.

The fake vehicle behind the unit and dry-run tests reproduces the PX4 behaviours the mission depends on, including two found in SITL: after an AUTO.LAND touchdown PX4 disarms and reverts to the last commanded mode that allows arming (OFFBOARD), and a fixed-wing RTL only lands if a mission landing pattern is loaded.

## Run

### Dry run (no PX4, no Gazebo)

```bash
ros2 launch autolivery_drone dry_run.launch.py rviz:=true
ros2 action send_goal --feedback /autolivery/deliver \
    autolivery_interfaces/action/Deliver "{location: rooftop_cafe}"
```

### Simulation (PX4 SITL + Gazebo)

```bash
ros2 launch autolivery_gazebo sim.launch.py vehicle:=quadcopter        # or vtol, fixed_wing
ros2 action send_goal --feedback /autolivery/deliver \
    autolivery_interfaces/action/Deliver "{location: park}"
```

`headless:=true` runs Gazebo without its GUI, `traffic:=false` removes the other aircraft, `planner:=rrt_star` switches planners, `rviz:=true` shows the map, route and active leg.

### Verified in PX4 SITL + Gazebo

PX4 v1.16.2, Gazebo Harmonic, MAVROS 2.15, headless:

| Vehicle | Delivery | Result |
|---|---|---|
| quadcopter (x500) | hotel_roof: 55 m roof, above cruise altitude | delivered, parcel 0.31 m from target on the roof, landed 0.26 m from the pad, 114 s |
| quadcopter (x500) | rooftop_cafe with traffic parked across the route | delivered; avoided the traffic on both legs, closest approach 5.4 m (4.6 m required) |
| vtol (standard_vtol) | farm, 515 m legs | delivered; PX4 log shows MC -> FW -> MC on each leg; parcel 1.4 m from target, landed 0.14 m from the pad, 251 s |
| fixed_wing (rc_cessna) | farm, air drop from the airstrip | runway takeoff, air drop 5.5-6.2 m from target, RTL flew the uploaded runway approach, landing detected and disarmed |

The Gazebo rc_cessna noses over during the rollout after that (otherwise clean) touchdown: PX4's attitude controller saturates the elevator on the ground before its land detector triggers, and the model's 3 cm wheels tip. That is the SITL airframe's ground handling, not the mission; see known gaps.

### Real vehicle

```bash
ros2 launch autolivery_drone autolivery.launch.py \
    vehicle:=hexacopter gripper:=winch fcu_url:=/dev/ttyACM0:921600 gcs_url:=udp://@10.0.0.2
```

Power the vehicle up on its depot pad (the autopilot's local origin is taken to be the depot), connect a ground station, and fly the first missions with a safety pilot on the RC transmitter: switching out of OFFBOARD hands control back immediately (see below).

Fixed-wings need a `runways` entry for their depot in the map. `mission_node` uploads it to PX4 as a mission landing (`DO_LAND_START`, loiter down to the approach altitude, straight-in `NAV_LAND`) as soon as MAVROS connects, and refuses deliveries until PX4 has accepted it; the flight ends with RTL, which flies that approach (`RTL_TYPE 1`, the PX4 fixed-wing default). Keep the glide slope under the airframe's `FW_LND_ANG`. Without a runway the plane falls back to PX4's circular `AUTO.LAND`.

`sim.launch.py` changes two PX4 parameters in simulation only: `NAV_DLL_ACT 0` (there is no ground station) and `FW_LND_USETER 0` (the Gazebo plane has no rangefinder, and PX4 aborts terrain-referenced landings without one). Real vehicles keep the airframe defaults: a GCS link, and a rangefinder on planes.

### Deliveries

```bash
# A named location from the map
ros2 action send_goal /autolivery/deliver autolivery_interfaces/action/Deliver "{location: hotel_roof}"
# An explicit point; snap_to_surface takes the drop height (ground or roof) from the map
ros2 action send_goal /autolivery/deliver autolivery_interfaces/action/Deliver \
    "{dropoff: {x: 120.0, y: 175.0}, snap_to_surface: true}"
# Check a route without flying it
ros2 service call /autolivery/plan_path autolivery_interfaces/srv/PlanPath \
    "{start: {x: 0, y: 0, z: 40}, goal: {x: 215, y: 78, z: 40}, planner: rrt_star}"
# Live state
ros2 topic echo /autolivery/mission/status
```

Refused deliveries come back as an aborted result with the reason (`the descent to the dropoff is blocked by park_tree_2`, `route is 9120 m but quadcopter range is 6000 m`, ...). Cancelling a goal in flight flies the vehicle home by a planned route and lands it.

## Safety behaviour

- **Everything is checked before the motors start**: takeoff corridor, the whole route out and back, the descent corridor over the dropoff, range, battery and parcel presence. Anything that fails is refused with a reason.
- **The autopilot outranks the mission.** If the flight mode leaves OFFBOARD without the mission asking (pilot takeover, PX4 geofence/RC/datalink/battery failsafe), the mission ends at once, stops streaming setpoints and stops requesting modes, so switching back to OFFBOARD later cannot send the vehicle to a stale target.
- **Battery**: below `battery_return` the delivery is abandoned and the vehicle flies home by a planned route; below `battery_land` it lands where it is.
- **Every wait has a deadline** (mode change, arming, climb, settle, release, each leg, landing). Aborts fly home by a planned route; the autopilot's straight-line RTL is only the last resort when no route can be planned.
- **Stuck parcel**: with a payload sensor, release is retried and, if it never comes off, the vehicle brings it home.
- **Stale data is missing data**: a pose older than 0.5 s or a state older than 2 s makes the vehicle "not ready" rather than trusted.
- **Traffic**: other aircraft on `/autolivery/obstacles` (ADS-B, simulated traffic, or your own perception) are avoided with a velocity-obstacle method that also refuses manoeuvres into buildings.

## The city map

`autolivery_drone/maps/urban_delivery.yaml` is the single source of truth. After editing it, regenerate the Gazebo world (the package test fails if you forget):

```bash
ros2_ws/src/autolivery_gazebo/scripts/generate_world.py \
    ros2_ws/src/autolivery_drone/maps/urban_delivery.yaml \
    ros2_ws/src/autolivery_gazebo/worlds/urban_delivery.sdf
```

For a real area, set `origin` to the depot's latitude/longitude/altitude and trace buildings, trees, masts and restricted airspace as boxes and cylinders in metres east/north of it.

## Status

| Area | State |
|---|---|
| Core planning, avoidance, mission state machine | complete, unit tested (20 mission scenarios, 8 profile/city deliveries) |
| ROS 2 nodes, launch, configs, dry run | complete, launch tested (quad + fixed-wing) |
| Gazebo world, PX4 vehicle + parcel models | complete |
| Quadcopter delivery in PX4 SITL + Gazebo | flown end to end, incl. traffic avoidance |
| VTOL delivery in PX4 SITL + Gazebo | flown end to end, incl. FW transitions |
| Fixed-wing air drop in PX4 SITL + Gazebo | flown end to end; sim airframe noses over on rollout |
| CI (GitHub Actions, ros:jazzy) | configured; runs build, all tests and linters |
| Hardware flight | not yet flown |

## Known gaps

- The world frame is tied to the autopilot's local origin, so the vehicle must be switched on at its depot. Dropoffs by latitude/longitude are not accepted yet (the map origin is there for ADS-B conversion).
- Air-drop release timing assumes a drag-free fall; parachuted parcels need a drag model.
- Dynamic obstacles come only from `/autolivery/obstacles`; there is no onboard perception (lidar/depth) node yet.
- Hexacopter, octocopter, coaxial and helicopter profiles have no Gazebo model; their mission logic is the multicopter path already tested in SITL.
- Gazebo's rc_cessna flips on the landing rollout (see "Verified in PX4 SITL"); the landing itself, detection and disarm are fine. A plane with more realistic landing gear in simulation, or real hardware, is needed to validate rollouts.
- The fixed-wing loiter-down point for the airstrip runway sits about 100 m west of the main (multirotor) depot at 40 m; with both fleets flying at once the two need separating in the map.
- The mission reports success on "landed and disarmed"; it does not yet read PX4's failure-detector state, so a hard landing that still ends disarmed counts as a completed delivery.
- `NAV_DLL_ACT` and `FW_LND_USETER` are overridden in simulation only (see "Real vehicle").
