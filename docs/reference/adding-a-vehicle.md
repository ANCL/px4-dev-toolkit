# Adding a vehicle

Vehicle YAML describes physical and PX4-specific facts about one aircraft/model. Controller gains belong elsewhere. Create `config/vehicles/<vehicle>.yaml`, trace each value to measurement or exact upstream source, and select only modes supported by the available data.

## 1. Identify the facts and source

Use the [existing files](https://github.com/ANCL/px4-dev-toolkit/tree/main/config/vehicles) as schema references. `gz_f450` has source-backed mass and hover-thrust data; `gz_fy690s` deliberately omits unvalidated hover thrust. Use the mass and geometry of the aircraft being added.

Record the source revision, airframe/model, total mass, coordinate frames, units, and calibration conditions. Treat simulator source data and measurements of a real vehicle as different evidence.

## 2. Match data to the command level

| Configuration | Required vehicle data |
|---|---|
| Acceleration | Positive finite `plant.mass_kg` |
| Attitude / attitude rate | Mass and positive finite `px4_allocator.hover_thrust` |
| Geometric normalized / PX4 rate | Mass and curated hover thrust |

The acceleration handoff requires mass. Attitude, body-rate, and the documented thrust/torque controllers also require a validated normalized hover-thrust coefficient. A mass value alone cannot enable modes requiring missing hover-thrust calibration.

The following **schema fragment** uses the actual FY690S mass. It illustrates an acceleration-only data subset; it is not calibration data for a new real aircraft:

```{ .yaml .no-copy }
vehicle:
  name: gz_fy690s
  type: hexarotor
plant:
  frame: FLU
  mass_kg: 3.70411
```

Mass loading reads `plant.mass_kg`; descriptive metadata preserves provenance. The complete checked-in file contains additional source-backed data. For another aircraft, derive its own values rather than copying these.

## 3. Validate vehicle parameters

Use `plant.mass_kg` for total vehicle mass in kilograms. The controller expects a positive finite value, and vehicle metadata should identify the aircraft/model and the source of its parameters.

For attitude, body-rate, geometric-normalized, and PX4-rate control, provide `px4_allocator.hover_thrust`, a positive finite coefficient consistent with the active PX4 vehicle configuration. Do not copy the F450 coefficient to another aircraft without verification.

Vehicle files may retain additional source-backed geometry, inertia, and propulsion data, but the five documented SE(3) configurations do not use that information to compute a physical body-moment command.

## 4. Build, select, and check

With a toolkit checkout, vehicle YAML is read on controller launch; restart the controller after edits. Rebuild in a **native repository-root development shell** when changing controller code:

```bash
./ros2/build.sh sitl
```

For a **prepared matching SITL aircraft**, select the existing FY690S acceleration-capable file in a **separate repository-root runtime shell**:

```bash
source ros2/runtime_env.sh
ros2 launch offboard_controllers se3.launch.py \
  vehicle:=gz_fy690s trajectory:=hover handoff:=acceleration
```

This command selects controller data; it does not select the Gazebo model or rewrite PX4 parameters. Configure/build the matching runtime separately before running it. Do not execute it against an F450 merely because the node accepts the YAML.

Check startup validation, frame signs, recorded mode/state and allocator/motor responses, and model/calibration agreement. Schema success and simulation compatibility are preliminary checks; physical flight requires the actual [hardware preparation](../environments/hardware.md).
