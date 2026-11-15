# Drone Ship Landing

Simulated autonomous landing of a multirotor on a moving platform aboard a moving ship, without GNSS in the final design. Built to learn the ROS 2, ArduPilot and Gazebo stack end to end, with the simulation kept as close to a real system as a desktop setup allows.

**Status: early.** The drone takes off, flies a short velocity-controlled leg and lands, commanded entirely from a ROS 2 node. Marker-based landing, the moving platform, the ship and GNSS-denied navigation are not implemented yet. See the roadmap.

## How it works

`takeoff_demo` is a ROS 2 node that talks to ArduPilot over its native DDS interface, with no MAVROS in between. It:

1. switches the autopilot to GUIDED mode,
2. arms the motors,
3. requests a takeoff to 5 m and waits until the altitude is reached,
4. publishes velocity commands for 5 s,
5. switches to LAND and exits once the drone is on the ground.

Each stage has a timeout. If the drone is airborne when a stage times out, the node commands a landing before exiting with an error.

| Component | Version used |
|-----------|--------------|
| Host | Windows 10 with WSL2 |
| Linux | Ubuntu 22.04 |
| ROS 2 | Humble |
| Simulator | Gazebo Harmonic |
| Autopilot | ArduCopter SITL, 4.8.0-dev (master) |
| Ground station | Mission Planner on Windows |

## Design decisions

- **ArduPilot SITL as the autopilot.** SITL runs the same flight code that runs on a real flight controller, so the control loops, estimator and failsafes in simulation are the real ones. It also pairs with Mission Planner, which I wanted to learn.
- **Native DDS instead of MAVROS.** ArduPilot exposes ROS 2 topics and services directly. Fewer moving parts, at the cost of a smaller interface than MAVLink offers.
- **Simulation time everywhere.** The simulator runs slower than real time on this machine, so every node takes its clock from the simulator (`use_sim_time:=true`). Durations in the code mean simulated seconds.
- **Built as a ladder.** Each roadmap step is a working demo on its own, so the project is showable at any point rather than only at the end.
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

**ROS 2 topic and service names.** On this ArduPilot build they live under `/ap/v1/`, not `/ap/` as in older documentation. Check with `ros2 service list -t | grep ap/`. The node takes the prefix as a parameter.

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

Terminal 1, the simulator:

```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

Terminal 2, once the drone is on the runway and reports "pre-arm good":

```bash
ros2 run ship_landing takeoff_demo --ros-args -p use_sim_time:=true
```

Optional, for watching status: `mavproxy.py --console --map --aircraft test --master=:14550`.

Parameters, all optional:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `prefix` | `/ap/v1` | Namespace of the ArduPilot DDS topics and services |
| `target_alt` | `5.0` | Takeoff altitude in metres |
| `cruise_speed` | `1.0` | Commanded speed during the cruise leg, m/s |
| `cruise_time` | `5.0` | Length of the cruise leg in seconds of simulation time |

Example: `ros2 run ship_landing takeoff_demo --ros-args -p use_sim_time:=true -p target_alt:=8.0`

## Known limitations and roadmap

- **Runs slower than real time.** On WSL2 with an RTX 3060 the simulation runs at roughly 0.5 to 0.7 times real time. ArduPilot and Gazebo advance in lockstep, so flight behaviour is unaffected, but it makes every test slower.
- **GPS is still in use.** The GNSS-denied part of the project starts at step 5.
- **The cruise leg is open-loop.** The node commands a velocity for a fixed time and does not check where the drone ends up.
- **No hardware validation.** Nothing here has flown on a real vehicle. The project reduces known contributors to the sim-to-real gap; it cannot measure the gap itself.

| Step | Goal | State |
|------|------|-------|
| 1 | ArduPilot SITL, Gazebo, ROS 2 and Mission Planner running together | Done |
| 2 | ROS 2 node commanding takeoff, velocity and landing | Done |
| 3 | Precision landing on a static fiducial marker, GPS still on | Next |
| 4 | Landing on a marker that moves on flat ground | Planned |
| 5 | GNSS switched off, estimator running on non-GPS sources | Planned |
| 6 | Ship with wave motion (heave, roll, pitch) and touchdown timing | Planned |
| 7 | Realism pass: sensor noise, latency, wind, batch runs with landing-error statistics | Planned |

## Repository layout

```
drone-ship-landing/
├── README.md
└── ship_landing/                 ROS 2 Python package
    ├── package.xml
    ├── setup.py
    └── ship_landing/
        └── takeoff_demo.py       takeoff, cruise and landing node
```

## Credits and AI assistance

This project stands on [ArduPilot](https://ardupilot.org) (ArduCopter SITL, the DDS interface and MAVProxy), [Gazebo](https://gazebosim.org) Harmonic, [ROS 2](https://docs.ros.org) Humble, the ArduPilot Gazebo integration packages [`ardupilot_gazebo`](https://github.com/ArduPilot/ardupilot_gazebo) and [`ardupilot_gz`](https://github.com/ArduPilot/ardupilot_gz), the Micro XRCE-DDS agent, and Mission Planner. The Iris quadcopter model and runway world come from those ArduPilot packages.

The project was built with Claude (Anthropic) as a coding assistant. Claude proposed the staged roadmap, wrote the `takeoff_demo` node and the first draft of this README, and diagnosed setup errors from the logs and screenshots I supplied. I chose the project and its goals, installed and built the whole stack, ran every command and every flight in the simulator, and decided what to keep. The problems under "Lessons learned" were found by running each step and reading its output, and the fixes listed are the ones that worked on my machine.

## License

MIT. See [LICENSE](ship_landing/LICENSE).
