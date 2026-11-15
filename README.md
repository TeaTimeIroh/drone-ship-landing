# Drone Ship Landing

Simulated autonomous landing of a multirotor on a moving platform aboard a moving ship, without GNSS in the final design. Built to learn the ROS 2, ArduPilot and Gazebo stack end to end, with the simulation kept as close to a real system as a desktop setup allows.

**Status: early.** The drone takes off, hovers and lands under the control of a ROS 2 node, and a second node finds a landing marker in the downward camera image and reports where it is. Steering the landing onto the marker, the moving platform, the ship and GNSS-denied navigation are not implemented yet. See the roadmap.

## How it works

`takeoff_demo` is a ROS 2 node that talks to ArduPilot over its native DDS interface, with no MAVROS in between. It:

1. switches the autopilot to GUIDED mode,
2. arms the motors,
3. requests a takeoff to 5 m and waits until the altitude is reached,
4. publishes velocity commands for 5 s,
5. switches to LAND and exits once the drone is on the ground.

Each stage has a timeout. If the drone is airborne when a stage times out, the node commands a landing before exiting with an error.

`marker_detector` reads the drone's downward camera, finds the pad's ArUco marker with OpenCV, and publishes the position of the marker centre relative to the camera, in metres. It uses the calibration the simulated camera publishes, and it also publishes the camera image with the detection drawn on it. The detection maths lives in `pad_detection.py`, which has no ROS dependency and its own tests.

The `landing_pad` world is the ArduPilot runway with a 1.25 m pad carrying a 1.0 m marker, placed 3 m east and 2 m north of the take-off point. The camera is the Iris model's gimbal camera, held pointing straight down by autopilot parameters in `config/camera_down.parm`.

| Component | Version used |
|-----------|--------------|
| Host | Windows 10 with WSL2 |
| Linux | Ubuntu 22.04 |
| ROS 2 | Humble |
| Simulator | Gazebo Harmonic |
| Autopilot | ArduCopter SITL, 4.8.0-dev (master) |
| Ground station | Mission Planner on Windows |
| Vision | OpenCV 5.0 (ArUco module) |

## Design decisions

- **ArduPilot SITL as the autopilot.** SITL runs the same flight code that runs on a real flight controller, so the control loops, estimator and failsafes in simulation are the real ones. It also pairs with Mission Planner, which I wanted to learn.
- **Native DDS instead of MAVROS.** ArduPilot exposes ROS 2 topics and services directly. Fewer moving parts, at the cost of a smaller interface than MAVLink offers.
- **Simulation time everywhere.** The simulator runs slower than real time on this machine, so every node takes its clock from the simulator (`use_sim_time:=true`). Durations in the code mean simulated seconds.
- **Built as a ladder.** Each roadmap step is a working demo on its own, so the project is showable at any point rather than only at the end.
- **ArUco marker, 4x4 dictionary.** The camera is wide-angle and only 640x480, so from 5 m a 1 m marker is about 40 pixels across. A 4x4 marker has the largest cells of the standard dictionaries, which keeps it readable at that size.
- **The landing controller will live in ROS 2, not in the autopilot.** ArduPilot has built-in precision landing, but it is fed over MAVLink, which the DDS interface does not cover. A controller in ROS 2 is also what the moving platform needs later. Comparing the two is future work.
- **No `cv_bridge`.** Images are converted to NumPy arrays by hand, which avoids a binary dependency that breaks when the system and pip versions of OpenCV and NumPy differ.
- **Versions follow ArduPilot's documentation.** Ubuntu 22.04, ROS 2 Humble and Gazebo Harmonic are the combination the ArduPilot guides describe, chosen over newer releases to stay on the supported path.

## Lessons learned

These cost time during setup and are not obvious from the guides.

**ArduPilot build fails with a missing `config.h.in`.** A git submodule was not checked out:

```bash
cd ~/ardu_ws/src/ardupilot
git submodule update --init --recursive --force
```

**Gazebo opens but no drone appears; the log shows `Unable to find uri[package://ardupilot_gazebo/...]`.** The model search path is missing one folder. Add to `~/.bashrc`:

```bash
source ~/ardu_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=$HOME/ardu_ws/install/ardupilot_gazebo/share:$GZ_SIM_RESOURCE_PATH
export SDF_PATH=$GZ_SIM_RESOURCE_PATH
```

**MAVProxy floods the terminal with "time moved backwards" (WSL2).** The Linux clock was jumping back by more than a second, several times a minute. Turning off the NTP service inside Ubuntu, so that Windows is the only time source, stopped it. I did not confirm the root cause beyond that.

```bash
sudo timedatectl set-ntp false
```

**Mission Planner cannot connect over UDP (WSL2 on Windows 10).** Connect with TCP to `127.0.0.1`, port `5762`.

**ROS 2 topic and service names change with a parameter.** They are `/ap/v1/...` when the autopilot parameter `DDS_USE_NS` is 1 and `/ap/...` when it is 0. Check with `ros2 service list -t | grep ap/`. After being caught by this, the nodes now look the prefix up when they start.

**The launch folder matters.** SITL keeps its saved parameters in `eeprom.bin` in whatever folder the simulator is launched from, and a saved value takes priority over the defaults files. Two launch folders held two different saved sets, so the same launch file produced different topic names. Always launch from the same folder; `param show <name>` in MAVProxy shows the value actually in use.

**Rebuild after editing.** Even with `colcon build --symlink-install`, an edited node kept running its old code until the package was rebuilt.

**Files copied in through Windows Explorer bring `:Zone.Identifier` files with them.** They are ignored in `.gitignore` now.

**Read the first error, not the last.** When the launch failed, hundreds of lines followed, but one early line named the missing file and everything after it was a consequence.

## Build and run

Follow the official ArduPilot guides in this order. They create the workspace `~/ardu_ws` used below.

1. [ROS 2 with SITL](https://ardupilot.org/dev/docs/ros2-sitl.html)
2. [ROS 2 with Gazebo](https://ardupilot.org/dev/docs/ros2-gazebo.html)

Then clone and build this package:

```bash
cd ~/ardu_ws/src
git clone https://github.com/TeaTimeIroh/drone-ship-landing.git
cd ~/ardu_ws
colcon build --packages-select ship_landing
source install/setup.bash
```

Terminal 1, the simulator with the landing pad. Launch it from `~/ardu_ws` every time:

```bash
cd ~/ardu_ws
ros2 launch ship_landing landing_pad.launch.py
```

Terminal 2, the marker detector:

```bash
ros2 run ship_landing marker_detector --ros-args -p use_sim_time:=true
```

Terminal 3, once the simulator log shows "EKF3 IMU0 is using GPS": take off and hover for 20 s, then land.

```bash
ros2 run ship_landing takeoff_demo --ros-args -p use_sim_time:=true -p cruise_speed:=0.0 -p cruise_time:=20.0
```

To watch the camera with the detection drawn on it:

```bash
ros2 run rqt_image_view rqt_image_view /landing_pad/debug_image
```

Optional, for watching autopilot status: `mavproxy.py --console --map --aircraft test --master=:14550`. Add `rviz:=true` to the launch command for RViz; it is off by default because it slows the simulation.

`takeoff_demo` parameters, all optional:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `prefix` | found at start-up | Namespace of the ArduPilot DDS topics and services, for example `/ap/v1` |
| `target_alt` | `5.0` | Takeoff altitude in metres |
| `cruise_speed` | `1.0` | Commanded speed during the cruise leg, m/s |
| `cruise_time` | `5.0` | Length of the cruise leg in seconds of simulation time |

Example: `ros2 run ship_landing takeoff_demo --ros-args -p use_sim_time:=true -p target_alt:=8.0`

`marker_detector` publishes `landing_pad/pose` (`geometry_msgs/PoseStamped`, marker centre in the camera optical frame: x right in the image, y down in the image, z out of the lens) and `landing_pad/debug_image`. Its parameters are `image_topic`, `camera_info_topic`, `marker_size` (metres, default 1.0), `marker_id` (default 0) and `publish_debug_image`.

### Tests

The detection code is tested against synthetic camera views with a known marker position. No simulator is needed:

```bash
cd ~/ardu_ws/src/drone-ship-landing/ship_landing
python3 -m pytest test/test_pad_detection.py
```

## Known limitations and roadmap

- **Runs slower than real time.** On WSL2 with an RTX 3060 the simulation runs at roughly 0.5 to 0.8 times real time. ArduPilot and Gazebo advance in lockstep, so flight behaviour is unaffected, but it makes every test slower.
- **GPS is still in use.** The GNSS-denied part of the project starts at step 5.
- **The cruise leg is open-loop.** The node commands a velocity for a fixed time and does not check where the drone ends up.
- **The marker is seen but not yet used.** The drone still lands where it took off, not on the pad.
- **Range is biased when the marker is small.** In the synthetic tests the position is good to a few centimetres within 5 m. Beyond about 8 m the marker is roughly 20 pixels wide and the range reads up to 5% long, while the direction stays within a degree. These figures come from rendered test images, not yet from the simulator.
- **Blind in the last half metre.** Below about 0.6 m the 1 m marker no longer fits in the image. A smaller marker for the final descent is planned.
- **No hardware validation.** Nothing here has flown on a real vehicle. The project reduces known contributors to the sim-to-real gap; it cannot measure the gap itself.

| Step | Goal | State |
|------|------|-------|
| 1 | ArduPilot SITL, Gazebo, ROS 2 and Mission Planner running together | Done |
| 2 | ROS 2 node commanding takeoff, velocity and landing | Done |
| 3a | World with a marker pad and a downward camera | Done |
| 3b | Detector node: marker position relative to the camera | Done |
| 3c | Landing controller: centre over the marker and descend, GPS still on | Next |
| 3d | Landing error measured over repeated runs | Planned |
| 4 | Landing on a marker that moves on flat ground | Planned |
| 5 | GNSS switched off, estimator running on non-GPS sources | Planned |
| 6 | Ship with wave motion (heave, roll, pitch) and touchdown timing | Planned |
| 7 | Realism pass: sensor noise, latency, wind, batch runs with landing-error statistics | Planned |

## Repository layout

```
drone-ship-landing/
├── README.md
└── ship_landing/                     ROS 2 Python package
    ├── package.xml
    ├── setup.py
    ├── launch/landing_pad.launch.py  simulator, autopilot and bridge in the pad world
    ├── worlds/landing_pad.sdf        runway world with the landing pad
    ├── models/aruco_pad/             the pad model and its marker texture
    ├── config/camera_down.parm       autopilot parameters: camera straight down
    ├── tools/make_marker.py          regenerates the marker texture
    ├── test/test_pad_detection.py    tests for the detection code
    └── ship_landing/
        ├── takeoff_demo.py           takeoff, hover or cruise, and landing node
        ├── marker_detector.py        camera image to marker position, ROS node
        ├── pad_detection.py          marker detection and pose estimate, no ROS
        └── ap_names.py               finds the autopilot's topic prefix
```

## Credits and AI assistance

This project stands on [ArduPilot](https://ardupilot.org) (ArduCopter SITL, the DDS interface and MAVProxy), [Gazebo](https://gazebosim.org) Harmonic, [ROS 2](https://docs.ros.org) Humble, the ArduPilot Gazebo integration packages [`ardupilot_gazebo`](https://github.com/ArduPilot/ardupilot_gazebo) and [`ardupilot_gz`](https://github.com/ArduPilot/ardupilot_gz), the Micro XRCE-DDS agent, and Mission Planner, and on [OpenCV](https://opencv.org) for marker detection. The Iris quadcopter model and the runway come from those ArduPilot packages. `worlds/landing_pad.sdf` is derived from `runway.sdf` in `ardupilot_gz`, which is licensed GPL-3.0.

The project was built with Claude (Anthropic) as a coding assistant. Claude proposed the staged roadmap, wrote the nodes, the launch file, the pad model, the tests and the first draft of this README, and diagnosed setup errors from the logs and screenshots I supplied. Several of its first diagnoses were wrong and were corrected by checking against what the simulator actually did. I chose the project and its goals, installed and built the whole stack, ran every command and every flight in the simulator, and decided what to keep. The problems under "Lessons learned" were found by running each step and reading its output, and the fixes listed are the ones that worked on my machine.

## License

MIT, see [LICENSE](ship_landing/LICENSE), except `ship_landing/worlds/landing_pad.sdf`, which is derived from a GPL-3.0 file as noted above.
