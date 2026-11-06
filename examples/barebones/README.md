# UWB plugin on its own

Use the UWB plugin in your own Gazebo world, without PX4 or ROS 2. It needs Gazebo Harmonic, CMake and a C++ compiler. See [Installing Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/).

## Run the example

From the repository root:

```bash
cmake -S uwb_gazebo_plugin -B build
cmake --build build -j
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build
gz sim -r examples/barebones/world.sdf
```

In another terminal, compare the two anchors. They are the same distance from the tag, but a wall blocks anchor 2:

```bash
gz topic -e -t /uwb_gz_simulator/distances/a1t1
gz topic -e -t /uwb_gz_simulator/distances/a2t1
```

Values are in centimetres. The true distances are on `/uwb_gz_simulator/distances_ground_truth/aItJ`, where `I` is the anchor id and `J` the tag id. In TDOA mode the topics are `/uwb_gz_simulator/tdoa/aIaKtJ` and `/uwb_gz_simulator/tdoa_ground_truth/aIaKtJ`, with `I` the lowest anchor id.

## Add it to your own world

1. Add the plugin inside `<world>`. To use a calibrated preset, generate the block and paste it in (this needs Python with PyYAML):

   ```bash
   python3 tools/preset_to_sdf.py eliko_identification
   ```

   The presets are `eliko_identification` (TWR) and `util_identification` (TDOA). Without a preset, the defaults are the Eliko line-of-sight calibration in TWR mode. See the [measurement model](../../docs/measurement_model.md) for every parameter.

2. Add links named `uwb_tag_<id>` and `uwb_anchor_<id>` to your robots, or to a static model for fixed anchors. Ids are integers, unique among tags and unique among anchors. On a moving robot, attach each one with a fixed joint. The `uwb_tag_eliko.stl` and `uwb_anchor_eliko.stl` meshes in [`models/uwb_meshes/`](../../models/uwb_meshes/) are optional, and so are the bars and poles. They are plain `<visual>` cylinders you add to the link yourself, as in [`world.sdf`](world.sdf).

3. Set `GZ_SIM_SYSTEM_PLUGIN_PATH` to the folder holding `libUWBGazeboPlugin.so`, or install it with `cmake --install build --prefix <dir>`.

## Rules your world must follow

- **Put UWB links directly on the robot's model.** In a nested model the robot isn't recognised, so its own links count as obstacles.
- **Name the robot's main link `base_link`** for it to shadow the signal. Other robot links are ignored.
- **Give obstacles `<collision>` geometry.** Objects with only a `<visual>` don't block the signal. Boxes are exact, and other shapes use their axis-aligned bounding box, which over-estimates them.
- **Name the floor `ground_plane`.** The plugin skips that exact name. A plane with any other name still works, but Gazebo logs a warning at every update.

To mark an obstacle's material, add a `__material_<name>` suffix to its model name, e.g. `wall_1__material_concrete`, and set `enable_material_attenuation` to `true`.

## Using it from ROS 2

The measurements are plain Gazebo topics of type `gz.msgs.Double`. Bridge the ones you need with [`ros_gz_bridge`](https://github.com/gazebosim/ros_gz):

```bash
ros2 run ros_gz_bridge parameter_bridge "/uwb_gz_simulator/distances/a1t1@std_msgs/msg/Float64[gz.msgs.Double"
```
