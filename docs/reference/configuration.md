<!-- Preserve existing section links. -->
<span id="configuration-ownership"></span>
<span id="rule-of-thumb"></span>

<span id="configuration-files"></span>
# Configuration

Find the file that owns a setting and when changes take effect.

<!-- Preserve existing section links. -->
<span id="ownership-map"></span>
<span id="configruntimecommonenv"></span>
<span id="configruntimesitlenv"></span>
<span id="configruntimehardwareenv"></span>
<span id="px4-topics"></span>

## Where to change a setting

<div class="px4-table-scroll" markdown="1">

| Setting | Canonical file or directory |
|---|---|
| Shared ROS distribution | [config/runtime/common.env](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/runtime/common.env) |
| SITL model, estimator, GCS and DDS | [config/runtime/sitl.env](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/runtime/sitl.env) |
| Hardware links, Vicon and transforms | [config/runtime/hardware.env](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/runtime/hardware.env) |
| Vehicle mass, inertia and propulsion | [config/vehicles](https://github.com/ANCL/px4-dev-toolkit/tree/main/config/vehicles) |
| Offboard SE3 gains and timing | [controller.yaml](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/config/se3/controller.yaml) |
| Onboard SE3 gains and trajectories | Native PX4 `SE3_*` / `TRJ_*` parameters and `MPC_THR_HOVER` |
| MAVLink Router | [config/mavlink-router.conf](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/mavlink-router.conf), `tools/mavlink_router` |
| ROS Offboard trajectory segments and names | [trajectories.yaml](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/trajectories.yaml) |
| Native SE3 trajectory geometry and durations | PX4 firmware `TRJ_*` parameters (QGroundControl or PX4 `param` shell) |
| Offboard staging | [offboard.yaml](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/px4_position_mode/config/offboard.yaml) |
| Ordered launch study | [config/sequences](https://github.com/ANCL/px4-dev-toolkit/tree/main/config/sequences) |
| PX4 topic symbols | [config/px4_topics.def](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/px4_topics.def) |
| Recorded topic selection | Shared [config/recording](https://github.com/ANCL/px4-dev-toolkit/tree/main/config/recording): `common.txt` plus the selected profile |
| External repository revisions | [config/sources](https://github.com/ANCL/px4-dev-toolkit/tree/main/config/sources) |
| Tested tool versions | [config/versions.env](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/versions.env) |

</div>

<!-- Preserve existing section links. -->
<span id="runtime-configuration"></span>
<span id="vehicle-configuration"></span>
<span id="controller-tuning"></span>
<span id="trajectories"></span>
<span id="sequences"></span>

## Selection and when changes take effect

Environment wrappers source their `.env` files when invoked. Changing a file does not update an already-running tmux process. Stop/restart at an appropriate time to apply service changes; do not stop a flight's required communication as a configuration shortcut.

The standard ROS 2 Offboard SE3 launch reads source controller, vehicle, and trajectory YAML from the toolkit checkout, then applies launch selections. Restart the controller after changing this YAML; no ROS rebuild is needed. A standalone installation without the checkout uses packaged YAML. The [vehicle guide](../extending/vehicles.md) distinguishes controller-data selection from simulator/PX4 airframe selection.

Native parameters live in PX4, not toolkit configuration files. Set them in
PX4's shell or QGroundControl, verify the values while disarmed, and explicitly
save them when persistence is required. See [Onboard control](../operations/onboard.md).

Changes to `config/trajectories.yaml` affect ROS Offboard controllers at their next launch. Native trajectories use independently configured firmware `TRJ_*` parameters, snapshotted on start. Both use absolute PX4 local NED; verify the coordinate origin before flight.

Sequences select launches and their arguments; the runner overrides recording output for its study. It does not change gains or calibrations. Controller launch-owned recordings snapshot their selected experiment configuration in `experiment.json`, so analysis can use recorded settings rather than current source YAML. Independent `tools/px4_record` recordings contain only profile/topic metadata; preserve the native parameter values and firmware revision separately. See [Recording](../experiments/recording.md) for the two contracts.

<!-- Preserve existing section links. -->
<span id="external-sources-and-versions"></span>

## Missing data and source pins

Vehicle data requirements are listed under [Offboard SE3](../controllers/offboard/se3/index.md#vehicle-support). Native SE3 reads PX4 parameters rather than ROS vehicle YAML. Set Vicon addresses and devices in `config/runtime/hardware.env`.

`config/sources/px4.env` selects the PX4 repository and ref for new installations; `config/sources/*.repos` pins the other fetched repositories. Existing PX4 checkouts are not reset automatically. `config/versions.env` records tool versions and managed download pins. See [Native SITL](../get-started/sitl/ubuntu.md#pinned-external-sources) and [Companion installation](../get-started/companion.md#install-on-the-companion-computer).

<!-- Preserve existing section links. -->
<span id="next"></span>

See [Troubleshooting](troubleshooting.md) when a valid configuration still behaves unexpectedly.
