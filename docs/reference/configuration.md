# Configuration files

Use this reference to find where a setting belongs. Physical facts, controller tuning, requested motion, and runtime connectivity have separate files so a change has a clear meaning.

## Where to change a setting

| Setting | Canonical file or directory |
|---|---|
| Shared ROS distribution | [config/runtime/common.env](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/runtime/common.env) |
| PX4 target/model/world, estimator overrides, GCS, UDP agent, session | [config/runtime/sitl.env](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/runtime/sitl.env) |
| Serial agent, Vicon server/object, frames/transforms, timestamps, session | [config/runtime/hardware.env](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/runtime/hardware.env) |
| Mass, inertia, COM, propulsion, allocator facts, hover scale | [config/vehicles](https://github.com/ANCL/px4-dev-toolkit/tree/main/config/vehicles) |
| SE(3) gains, rate reproduction parameters, outer/warmup/retry timing | [controller.yaml](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/config/se3/controller.yaml) |
| Trajectory segments and names | [trajectories.yaml](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/config/trajectory/trajectories.yaml) |
| Position-mode staging settings | [offboard.yaml](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/px4_position_mode/config/offboard.yaml) |
| Ordered launch study | [config/sequences](https://github.com/ANCL/px4-dev-toolkit/tree/main/config/sequences) |
| PX4 topic symbols | [config/px4_topics.def](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/px4_topics.def) |
| Recorded topic selection | Package `config/recording/` and Position-reference `recording_topics.txt` |
| External repository revisions | [config/sources](https://github.com/ANCL/px4-dev-toolkit/tree/main/config/sources) |
| Validated host/tools and managed application versions | [config/versions.env](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/versions.env) |

## Selection and when changes take effect

Environment wrappers source their `.env` files when invoked. Changing a file does not update an already-running tmux process. Stop/restart at an appropriate time to apply service changes; do not stop a flight's required communication as a configuration shortcut.

The standard SE(3) launch loads the installed controller tuning, then supplies vehicle, trajectory, command-level, direct-controller, and diagnostic launch selections. Vehicle and trajectory source data are installed into the package during the ROS build; rebuild after changing them. The [vehicle guide](adding-a-vehicle.md) distinguishes controller-data selection from simulator/PX4 airframe selection.

Sequences select launches and their arguments; the runner overrides recording output for its study. It does not change gains or calibrations. Keep the configuration used with the recorded data so later trajectory-aware analysis does not read changed maneuver definitions.

## Missing data and source pins

Missing hover calibration or physical propulsion data deliberately limits supported modes. Do not invent values to pass validation. Gains belong in controller tuning rather than vehicle files; Vicon host/device identifiers belong in the site-specific runtime configuration rather than public walkthroughs.

`config/sources/*.repos` specifies what exact revisions to fetch. `config/versions.env` records tested tools and managed download pins; it is not a declaration that arbitrary installed versions were automatically verified. Source fetch behavior is described in [native installation](../installation/native.md#pinned-external-sources).

## Documentation media

The homepage currently has two static, clearly labeled 16:9 media slots. Replace their asset references only with approved SITL/lab footage. Short muted WebM/MP4 loops can use `playsinline`, a still poster, conservative preload, and pause/play controls; reduced-motion users must see the still image without automatic movement. No site content should depend on playback. The present slots intentionally have no moving media or playback controls.

Continue with [Troubleshooting](troubleshooting.md) for a setting that is accepted but behaves unexpectedly.
