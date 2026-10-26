# Adding a vehicle

Add `config/vehicles/<vehicle>.yaml` for the aircraft's physical parameters and PX4 allocator calibration. Use the [existing vehicle files](https://github.com/ANCL/px4-dev-toolkit/tree/main/config/vehicles) as schema examples.

## 1. Define the vehicle data

Define the aircraft identity, frame, and measured mass. For example, this is the file structure with placeholders for vehicle-specific values:

```{ .yaml .no-copy }
vehicle:
  name: <vehicle_name>
  type: <aircraft_type>
plant:
  frame: FRD
  mass_kg: <total_mass_kg>
```

Add available calibration and geometry fields using the existing files as a schema reference.

<div class="px4-table-scroll" markdown="1">

| Data | What to establish |
|---|---|
| `vehicle` | Aircraft identity, model and provenance |
| `plant.mass_kg` | Positive, finite **total** mass in kilograms |
| `plant.frame` | Frame used for physical plant quantities (`FLU` or `FRD`) |
| `px4_allocator.hover_thrust` | Positive, finite normalized hover-thrust calibration consistent with the active PX4 configuration, when available |
| Inertia and center of mass | Source-backed mass distribution and reference point; inertia must be positive definite |
| Propulsion and allocator geometry | Rotor positions, thrust axes, yaw-moment signs/ratios, actuator-thrust curve and matching PX4 allocation parameters |

</div>

Use the detailed propulsion/allocator keys in `gz_f450` when those parameters are available. [Offboard SE3](../controllers/offboard/se3/index.md#vehicle-support) specifies the required fields for each handoff.

## 2. Check frames and geometry

`plant.frame` and `propulsion.frame` can be FLU or FRD; the loader converts physical vectors and inertia to FRD. `px4_allocator.frame` must be FRD. In particular, the inertia conversion is

\[
J_{FRD}=C J_{FLU} C^T,\qquad C=\operatorname{diag}(1,-1,-1).
\]

Convert values when changing frames. Check rotor indices and geometry signs, thrust-curve monotonicity and invertibility, and correspondence with the active PX4 allocator. The current physical-wrench adapter supports **four rotors**; see [Geometric (Physical)](../controllers/offboard/se3/geometric-physical.md#vehicle-data-and-limits).

Match the parameters against the active simulator model or aircraft and PX4 configuration.

## 3. Load and check

Check mass, inertia, units, frame conversions, rotor geometry, and calibration against the selected aircraft. The ROS Offboard controller reads vehicle YAML at startup; restart it after edits. Native PX4 SE3 uses firmware parameters, not this YAML. See [Offboard SE3](../controllers/offboard/se3/index.md#run) for selection and [Companion computer](../operations/companion.md#before-flight) for physical-flight checks.
