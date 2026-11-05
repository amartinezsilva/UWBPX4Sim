# UWBPX4Sim

:pushpin: This work has been presented at the [1st Workshop on Robot Meets GNSS and Ranging for Seamless Autonomy](https://robotmeetsranging.tech/) during ICRA 2026!

:pushpin: This work has been featured in the [ROSCon Spain 2026](https://roscon.org.es/roscon2026/ROSConES2026.html) official program!

![](images/Logo.png)

A ready-to-use Ultra-Wideband simulator for relative localization of multi-robot teams integrated with ROS 2, Gazebo Harmonic and PX4 SITL. This is a standalone repo that is complementary to the work [Radio-based Multi-Robot Odometry and Relative Localization](https://github.com/amartinezsilva/mr-radio-localization?tab=readme-ov-file), containing all simulation-based tools used in the main work.

![](images/SimExtendedDiagram.drawio.png)

## Features

- **UWB measurement plugin** for Gazebo: two-way ranging (TWR) or time difference of arrival (TDOA), with noise, dropout and bias driven by distance and by the obstacles in the way, including their material. See [Measurement model](docs/measurement_model.md).
- **Calibrated presets**: `eliko_identification` (Eliko, TWR) and `util_identification` (UTIL dataset, TDOA). Tune one and save your own from the setup GUI.
- **Multi-robot layouts** from one YAML file: UAVs with tags, UGVs with anchors, and anchors fixed in the world. Vehicle models and the ROS 2 bridge are generated from it.
- **Guided setup** with a browser GUI, and a one-command **launcher**.
- **ROS 2 offboard nodes** that fly the UAVs and drive the UGVs along CSV trajectories.

## Getting started

A `docker/` folder is provided with a ready-to-use image (Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, PX4, and this repo's plugin/bridge pre-built) as an alternative to setting everything up natively. It requires [Docker](https://docs.docker.com/engine/install/) and the [Docker Compose plugin](https://docs.docker.com/compose/install/).

```bash
cd <ros_ws>/src
git clone --recurse-submodules https://github.com/amartinezsilva/UWBPX4Sim.git
cd UWBPX4Sim/docker
./setup.sh
```

The first build compiles the PX4 toolchain and the full ROS 2 workspace, so it can take a while (15-30 minutes depending on your machine); subsequent runs reuse the cached image and start instantly. `setup.sh` then starts a **persistent** container in the background and drops you into its guided menu:

1. **Run UWBPX4Sim setup** (`setup_simulator.sh`): configure the PX4/Gazebo plugin, models, and layout inside the container. Opens a browser-based GUI (prints the URL, default `http://localhost:5050`) to build the robot layout and tune plugin parameters; falls back to terminal prompts if the GUI isn't available.
2. **Launch simulation** (ticks once setup has been run): launches PX4 SITL, Gazebo, and the UWB bridge/offboard nodes via `simulator_launcher.sh`.
3. **Open a shell**: skip the menu and get a plain shell in the container.
4. **Exit**: stops the container, and asks whether to remove it completely instead (default just stops it) -- everything else, including closing your terminal, leaves it running with your configuration intact. Run `./setup.sh` again any time to get back to this same menu.

`docker-compose.yml` bind-mounts the whole repository (plus `ROS2/px4_sim_offboard/trajectories/` onto its installed path) from the host into the container, so editing a layout, a plugin-parameter preset, a custom world, a trajectory CSV, or the plugin source on the host takes effect the next time setup (or the simulation) runs -- no image rebuild needed.

This image is also the base that [mr-radio-localization](https://github.com/amartinezsilva/mr-radio-localization)'s own Docker image builds `FROM`, adding its radar/localization stack on top -- see that repo's `docker/` for the layered setup.

To run without Docker, see [Running without Docker](docs/advanced_setup.md#running-without-docker).

## Configuring an experiment

A layout YAML in `config/` defines the robots, their spawn poses, the UWB sensors they carry and their trajectories:

```yaml
world: walls_nlos          # optional: a file from worlds/, or a world PX4 ships
uavs:
  - id: 0
    spawn_pose: [3.0, 0.0, 0.0, 0.0, 0.0, 1.57079632679]   # x, y, z, roll, pitch, yaw
    offboard:
      trajectory_csv_file: trajectory_lemniscate_uav.csv
    tags:
      - id: 1
        position: [-0.24, -0.24, -0.06]                    # relative to the robot
ugvs:
  - id: 0
    spawn_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 1.57079632679]
    offboard:
      trajectory_csv_file: trajectory_lemniscate_agv.csv
    anchors:
      - id: 1
        position: [-0.32, 0.3, 0.875]
static_anchors:                                            # optional, not on a robot
  - id: 5
    position: [4.0, 2.0, 2.5]                              # in world coordinates
```

Tag ids must be unique across all UAVs, and anchor ids across all UGVs and static anchors. `ugvs` is optional when there are static anchors. See [`config/`](config/) for complete examples. Build layouts from the setup GUI or edit the files directly, then run setup again.

To mark an obstacle's material, add a `__material_<name>` suffix to its model name in the world, e.g. `wall_1__material_concrete`.

## Output topics

| Mode | Measurements | Ground truth |
| --- | --- | --- |
| TWR | `/uwb_gz_simulator/distances/aItJ` | `/uwb_gz_simulator/distances_ground_truth/aItJ` |
| TDOA | `/uwb_gz_simulator/tdoa/aIaKtJ` | `/uwb_gz_simulator/tdoa_ground_truth/aIaKtJ` |

`I` and `K` are anchor ids (`I` is the TDOA reference, the lowest anchor id) and `J` is the tag id. All of these topics are bridged to ROS 2, but only the active mode publishes. Each robot also publishes `/<prefix>/odom` and `/<prefix>/gt`, with prefix `uav_<id>` or `ugv_<id>`.

## Documentation

- [Measurement model](docs/measurement_model.md): equations, materials, all plugin parameters and calibration results.
- [Advanced setup](docs/advanced_setup.md): running without Docker, full layout reference, adding vehicle types, manual installation and ROS 2 interfaces.
