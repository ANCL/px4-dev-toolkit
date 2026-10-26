<span id="tools-and-command-interfaces"></span>

<span id="tools-and-terminals"></span>

# Tools

Run toolkit commands from the repository root. In Docker, run Docker commands on the host and toolkit/ROS commands inside the container.

## Choose a terminal

<div class="px4-table-scroll" markdown="1">

| Terminal | Open it | Use it for |
|---|---|---|
| Ubuntu shell | Repository root | `./tools/…`, ROS 2 launches, recording and analysis |
| PX4 in SITL | `px4` tmux window | Native PX4 commands, including `se3_control` and `trajectory` |
| PX4 on hardware | `px4-shell` tmux window or `./tools/px4_shell` | Native commands on the flight controller |
| Ground control | SITL MAVProxy/QGC; hardware GCS | Pilot inputs, status and mode changes |

</div>

PX4 uses `pxh>` in SITL and typically `nsh>` on hardware. Ubuntu and PX4 commands are not interchangeable. The `px4-sitl` and `px4-hardware` sessions contain the named service windows; use a separate shell for manual ROS commands.

## Start and inspect services

Choose the environment installed on this machine:

```bash
./tools/sitl start headless
```

or, on the physical companion:

```bash
./tools/hardware start
```

SITL starts Gazebo with a GUI unless `headless` is selected. Repeating `start` attaches to the running session. Use `./tools/sitl status` or `./tools/hardware status` in another shell; use the matching `stop` after landing and disarming.

The [SITL](../environments/sitl.md) and [companion](../environments/hardware.md) guides list the services and configuration for each runtime.

## Communication paths

**MAVLink Router** is a background transport service, independent of both
ground control and DDS. In SITL it uses PX4's API stream at `127.0.0.1:14540`;
MAVProxy uses the separate GCS stream at port 14550. The router supplies local
UDP endpoints 14661 for the PX4 shell and 14662 for parameter/trajectory tools.
On hardware it owns the configured MAVLink serial device, separate from the
DDS serial device. Optional `MAVLINK_QGC_OUT` adds GCS forwarding. Use one
toolkit router with these local ports at a time.

**uXRCE-DDS** transports PX4 state and ROS 2 controller setpoints through the
agent (UDP in SITL, serial in the hardware profile). Source the ROS runtime in
each manual ROS shell:

```bash
source ros2/runtime_env.sh
./tools/topic_rates vehicle_local_position vehicle_status_v1 --duration 5
```

`topic_rates` measures host callback arrivals after a 1-second warmup; these are not controller update rates. The hardware mocap bridge provides estimator observations, and trajectory uploads use MAVLink FTP.

SITL's default MAVProxy loads a software joystick used by the repeatable
Position-mode takeoff/staging demonstrations. QGroundControl is an optional
SITL frontend; choosing it does not provide that software joystick. Physical
aircraft use their configured pilot controls and independent GCS as required.

## Command and tool reference

The **Ubuntu repository shell** provides the following toolkit commands; commands that contact PX4 require the matching environment, router connection, and vehicle state.

<div class="px4-table-scroll" markdown="1">

| Tool | Purpose | Procedure |
|---|---|---|
| `./tools/onboard_params` | Apply and verify a named PX4 parameter profile while disarmed | [Onboard control](onboard.md#prepare-before-takeoff) |
| `./tools/trajectory` | List/compile YAML plans, upload them over MAVLink FTP, or explicitly clean plan files | [Trajectories](../controllers/trajectories.md#compile-and-upload-an-onboard-plan) |
| `./tools/px4_record` | Independently record PX4 DDS telemetry to a ROS bag | [Onboard logging](../workflows/onboard-logging.md) |
| `./tools/px4_analyze` | Analyze a finalized ROS bag or PX4 ULog | [Analysis](../analysis/analysis.md) |
| `./tools/px4_compare` | Compare supported recorded ROS flights | [Comparison](../analysis/comparison.md) |
| `./tools/run_sequence` | Run a configured series of controller launches | [Sequences](../workflows/sequences.md) |

</div>

Parameter loading is **not transactional**: check readbacks after failures. `./tools/trajectory clean` deletes both compiled local plans and uploaded plans. The `gz_f450` parameter profile is for SITL. Follow [Onboard control](onboard.md#se3-example-native-f450-sitl) for the flight procedure.

In the **PX4 shell**, use native `se3_control` and `trajectory` commands. See [Onboard](onboard.md) and [Offboard](offboard.md) operation.

## Recording and analysis tools

`./tools/px4_record onboard_se3` starts independent ROS recording in the foreground; the recorder does not arm, select a mode or start motion. `./tools/px4_record status` and `stop` can be used from another repository shell. A supported ROS launch can instead own recording with `record:=true`. PX4 ULog is a separate on-aircraft log, transferred after finalization.

`px4_analyze` reads ROS bags or `.ulg` files. `px4_compare` compares finalized ROS bags and requires `--output` for standalone runs outside a recognized study. ULog comparison uses a separate command. See [Data formats](../workflows/common-analysis.md), [Recording](../workflows/recording.md), and [Analysis](../analysis/analysis.md). `run_sequence` launches controllers and requires prepared SITL.
