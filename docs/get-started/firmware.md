# PX4 firmware

Build and flash the lab PX4 firmware from an **Ubuntu 24.04 workstation** using the terminal. Firmware runs on the flight controller; the [companion setup](companion.md) installs ROS 2 and communication services separately.

The commands below use the toolkit's configured [PX4 source](https://github.com/ANCL/px4-dev-toolkit/blob/main/config/sources/px4.env) (`ANCL/PX4-Autopilot`, `lab/main`). The FMUv6C target is an example for a **confirmed FMUv6C flight controller**; the F450 airframe name alone does not identify the board. Do not flash a firmware target meant for a different board.

## 1. Prepare the PX4 source

From the **toolkit repository root on the build workstation**:

```bash
source config/sources/px4.env
```

For an existing checkout (as installed for native SITL), check for local changes, then update the branch and submodules:

```bash
cd px4/PX4-Autopilot
git status --short --branch
git fetch origin --prune
git switch lab/main
git merge --ff-only origin/lab/main
git submodule update --init --recursive
git rev-list --left-right --count HEAD...origin/lab/main
```

The final comparison should be `0 0`. **Stop if there are local modifications or divergent commits**; review them rather than resetting the checkout. The toolkit's setup scripts do not automatically update an existing PX4 source tree.

If PX4 has **not** been cloned into this toolkit checkout, start instead at its repository root and obtain the selected source:

```bash
source config/sources/px4.env
mkdir -p px4
git clone --branch "$PX4_REF" "$PX4_REPO" px4/PX4-Autopilot
cd px4/PX4-Autopilot
git submodule update --init --recursive
```

## 2. Install the hardware build dependencies

On the **Ubuntu build workstation**, install the NuttX/ARM build tools once:

```bash
bash Tools/setup/ubuntu.sh --no-sim-tools
arm-none-eabi-gcc --version
```

A successful compiler version check confirms that the ARM compiler is available. The toolkit's SITL dependency installer uses `--no-nuttx`, so an existing SITL installation may not have the firmware toolchain. Follow any reboot guidance shown by the installer.

## 3. Build the firmware

Confirm the board model before choosing its target. If the existing PX4 firmware is running, `ver hw` in the PX4 shell can help identify the board; also check the controller model against its manufacturer documentation. For a **PX4 FMUv6C** controller:

```bash
make px4_fmu-v6c_default
```

Check the resulting file:

```bash
ls -lh build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4
```

This board configuration includes native `se3_control` and its built-in trajectory sampler. A successful build is not proof that the firmware was flashed or that the vehicle is ready to fly.

## 4. Flash through USB

Remove the propellers, ensure the aircraft is disarmed, and disconnect the flight battery and any other power sources. Use a direct USB connection between the flight controller and the build workstation, with the cable initially **unplugged from the controller**. Close any program occupying its USB serial port. Preserve the current vehicle parameters/configuration before the update so they can be checked afterward.

From the same **PX4 source directory**:

```bash
make px4_fmu-v6c_default upload
```

The uploader builds if needed and then searches for the PX4 bootloader. When it prints `waiting for the bootloader`, **plug in the controller USB cable**. If it continues waiting, unplug and reconnect **USB** to re-enter the bootloader; do not interrupt an active erase/program/verify step.

A successful upload reports `Erase`, `Program`, and `Verify` at 100%, then `Rebooting` and `Built target upload`. An unsuccessful build or upload must be resolved before proceeding.

## 5. Verify the installed firmware

After the flight controller reboots, reconnect using the normal PX4 shell interface (on physical hardware, the toolkit provides `./tools/px4_shell` through the configured MAVLink connection). **These are PX4 shell commands, not Ubuntu commands**:

```text
ver hw
ver git
se3_control status
param show TRJ_F8_T
```

Confirm the hardware identity and expected firmware revision, then confirm the onboard module is present. On hardware, an included module may not be automatically started; if SE3 is stopped, `se3_control start` is a **disarmed-only** registration action. Check `se3_control status` again before attempting to select its flight mode.

Check that the aircraft's parameters, airframe configuration, sensors, actuators, estimator, pilot controls, and failsafes are correct before any physical flight. Firmware flashing does not substitute for these checks. Continue with [Companion operation](../operations/companion.md) and [Onboard control](../operations/onboard.md) only after the aircraft is configured and verified.

For build targets and terminal uploading details, see [PX4's official build guide](https://docs.px4.io/main/en/dev_setup/building_px4).
