# Adding a vehicle

Vehicle YAML describes physical and PX4-specific facts about one aircraft/model. Controller gains belong elsewhere. Create `config/vehicles/<vehicle>.yaml`, trace each value to measurement or exact upstream source, and select only modes supported by the available data.

## 1. Identify the facts and source

Use the [existing files](https://github.com/ANCL/px4-dev-toolkit/tree/main/config/vehicles) as schema references. `gz_f450` has a curated physical propulsion model; `gz_fy690s` deliberately omits unvalidated hover thrust. Neither is a requirement to imitate another airframe's mass or geometry.

Record the source revision, airframe/model, total mass, coordinate frames, units, and calibration conditions. Physical inertia should describe the complete vehicle about its center of mass, with the same body convention used by the law. Treat simulator source data and measurements of a real vehicle as different evidence.

## 2. Match data to the command level

| Configuration | Required vehicle data |
|---|---|
| Acceleration | Positive finite `plant.mass_kg` |
| Attitude / attitude rate | Mass and positive finite `px4_allocator.hover_thrust` |
| Geometric normalized / PX4 rate | Mass and curated hover thrust |
| Geometric physical | Mass; COM and positive-definite inertia; four-rotor propulsion and actuator curve; matching PX4 allocator mapping |

The physical path does not require hover thrust; the normalized-thrust paths do not consume inertia to create a physical moment. A new mass file cannot enable modes that need missing calibration.

The following **schema fragment** uses the actual FY690S mass. It illustrates an honest acceleration-only data subset; it is not calibration data for a new real aircraft:

```{ .yaml .no-copy }
vehicle:
  name: gz_fy690s
  type: hexarotor
plant:
  frame: FLU
  mass_kg: 3.70411
```

Mass loading reads `plant.mass_kg`; descriptive metadata preserves provenance. The complete checked-in file contains additional source-backed data. For another aircraft, derive its own values rather than copying these.

## 3. Supply frames and actuator mapping

Physical `plant` data includes `frame`, `center_of_mass_m` with x/y/z keys, and symmetric `inertia_kg_m2` with xx/xy/xz/yy/yz/zz. The physical loader accepts FLU or FRD, transforms vectors and inertia to FRD ($D=\operatorname{diag}(1,-1,-1)$, $J_{FRD}=D J_{FLU}D^\top$), and subtracts the COM from rotor positions.

`propulsion` supplies frame, `rotor_count`, positive `moment_ratio_m`, indexed rotor `position_m`/`direction`, and `actuator_thrust` with `control_min`, `control_max`, and `polynomial_n` ordered $[c_0,c_1,c_2]$. The curve must be nonnegative, finite, and strictly increasing on the chosen subset of [0,1]. The loader expects unique rotor indexes 0–3 and `cw`/`ccw` direction strings.

For reference, this **F450 fragment** shows a real actuator curve, not a complete vehicle file:

```{ .yaml .no-copy }
propulsion:
  frame: FLU
  rotor_count: 4
  moment_ratio_m: 0.0137
  actuator_thrust:
    control_min: 0.0
    control_max: 1.0
    polynomial_n: [0.27, 3.06, 8.67]
```

`px4_allocator` requires FRD geometry, airframe type 0, four indexed rotors, positive thrust coefficient, rotor positions and `km`. The loader supports a rotor-specific `ct` or the common `thrust_coefficient`. These must describe **PX4's active configuration**, which may differ numerically from the physical geometry. Consult [loader.cpp](https://github.com/ANCL/px4-dev-toolkit/blob/main/ros2/src/offboard_controllers/src/se3/physical_wrench/config/loader.cpp) and [gz_f450.yaml](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/vehicles/gz_f450.yaml) for the complete schema.

The [physical adapter](../controllers/se3/geometric-physical.md) is fixed at four rotors. A six-rotor YAML can support acceleration and, with verified hover data, normalized modes; it cannot create a generic physical adapter. Other thrust-axis/allocator designs also require implementation analysis.

## 4. Build, select, and check

Vehicle files are installed into the controller package by CMake. In a **native repository-root development shell**, rebuild after changing source data:

```bash
./ros2/build.sh
./ros2/test.sh
```

For a **prepared matching SITL aircraft**, select the existing FY690S acceleration-capable file in a **separate repository-root runtime shell**:

```bash
source ros2/runtime_env.sh
ros2 launch offboard_controllers se3.launch.py \
  vehicle:=gz_fy690s trajectory:=hover handoff:=acceleration
```

This command selects controller data; it does not select the Gazebo model or rewrite PX4 parameters. Configure/build the matching runtime separately before running it. Do not execute it against an F450 merely because the node accepts the YAML.

Check startup validation, frame signs, physical thrust feasibility/matrix conditioning where relevant, recorded mode/state and allocator/motor responses, and model/calibration agreement. Schema success and simulation compatibility are preliminary checks; physical flight requires the actual [hardware preparation](../environments/hardware.md).
