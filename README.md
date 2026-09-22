# F450 PX4 + Gazebo Setup

This README explains how to add and run the custom F450 quadcopter model in PX4 SITL with Gazebo.

## 1. Requirements

You need:

* Ubuntu with PX4 SITL installed
* PX4-Autopilot
* Gazebo Sim
* QGroundControl (optional, but recommended)

The setup was developed and tested with the PX4 SITL workflow.

---

# 2. Copy the F450 Gazebo Model

Copy the `f450` model folder into:

```text
PX4-Autopilot/Tools/simulation/gz/models/
```

The resulting structure should be:

```text
PX4-Autopilot/
└── Tools/
    └── simulation/
        └── gz/
            └── models/
                └── f450/
                    ├── F450 Quadcopter Frame with Pixhawk 2.4.8 Flight Controller.mtl
                    ├── F450 Quadcopter Frame with Pixhawk 2.4.8 Flight Controller.obj
                    ├── f450_test.sdf
                    ├── model.config
                    └── model.sdf
```

The important files are:

* `model.sdf` — F450 Gazebo model
* `model.config` — Gazebo model information
* `f450_test.sdf` — test world/setup
* `.obj` and `.mtl` — CAD visual files

---

# 3. Add the PX4 Airframe

Copy:

```text
22000_gz_f450
```

into:

```text
PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
```

The resulting path should be:

```text
PX4-Autopilot/
└── ROMFS/
    └── px4fmu_common/
        └── init.d-posix/
            └── airframes/
                └── 22000_gz_f450
```

This file tells PX4 how the F450's motors are arranged and how the custom Gazebo model should be used.

---

# 4. Register the Airframe in CMake

Open:

```text
PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
```

Inside the `px4_add_romfs_files()` section, add:

```text
22000_gz_f450
```

For example:

```text
70000_gz_atmos
70001_gz_atmos_dual

22000_gz_f450

# [22000, 22999] Reserve for custom models
```

This allows PX4 to include the new airframe during the build.

---

# 5. Build PX4

Go to the PX4 directory:

```bash
cd ~/PX4-Autopilot
```

Build and launch the F450:

```bash
make px4_sitl gz_f450
```

PX4 should build the custom airframe and start Gazebo with the F450 model.

---

# 6. Expected Gazebo Model

The F450 uses an X configuration.

Top view:

```text
             FRONT
               ↑

          2          0

          3          1

               ↓
              REAR
```

The four motor positions are approximately 159.1 mm from the center in both X and Y directions.

The physical motor positions in Gazebo are:

```text
motor 0:  (+0.1591, +0.1591)
motor 1:  (+0.1591, -0.1591)
motor 2:  (-0.1591, +0.1591)
motor 3:  (-0.1591, -0.1591)
```

The F450 wheelbase is approximately:

```text
450 mm
```

---

# 7. PX4 Motor Geometry

PX4 uses a different body-frame convention from Gazebo.

Therefore, the PX4 rotor positions use the corresponding Y-axis conversion.

The airframe contains:

```text
CA_ROTOR0_PX  0.1591
CA_ROTOR0_PY -0.1591

CA_ROTOR1_PX  0.1591
CA_ROTOR1_PY  0.1591

CA_ROTOR2_PX -0.1591
CA_ROTOR2_PY -0.1591

CA_ROTOR3_PX -0.1591
CA_ROTOR3_PY  0.1591
```

The rotor directions and torque coefficients are also defined in the airframe file.

---

# 8. Motor/Gazebo Interface

The Gazebo motor plugins use the namespace:

```text
/f450_0
```

The PX4 motor commands are sent through:

```text
/f450_0/command/motor_speed
```

The four motors are connected to the corresponding propeller joints.

---

# 9. PX4 Control Allocation

The F450 uses PX4 Control Allocation.

The relevant rotor parameters are:

```text
CA_ROTOR_COUNT = 4
```

and the individual:

```text
CA_ROTOR*_PX
CA_ROTOR*_PY
CA_ROTOR*_KM
```

parameters.



---

# 10. Important PX4/Gazebo Frame Convention

Gazebo uses an ENU world convention:

```text
X = East
Y = North
Z = Up
```

PX4 uses NED:

```text
X = North
Y = East
Z = Down
```

For the vehicle body frame:

Gazebo:

```text
FLU
Front - Left - Up
```

PX4:

```text
FRD
Front - Right - Down
```

Therefore, when transferring horizontal motor positions between Gazebo and PX4, the Y direction needs to be accounted for.

This is why the PX4 `CA_ROTOR*_PY` values do not have the same signs as the Gazebo motor positions.

---


# 11. File Summary

The F450-specific files are:

```text
Tools/simulation/gz/models/f450/
├── F450 Quadcopter Frame with Pixhawk 2.4.8 Flight Controller.mtl
├── F450 Quadcopter Frame with Pixhawk 2.4.8 Flight Controller.obj
├── f450_test.sdf
├── model.config
└── model.sdf

ROMFS/px4fmu_common/init.d-posix/airframes/
└── 22000_gz_f450
```

Additionally, the following existing PX4 file needs the F450 airframe registration:

```text
ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
```

with:

```text
22000_gz_f450
```

---

# 14. Launch Command

Once the setup has been installed into a PX4-Autopilot source tree:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_f450
```

This builds and launches the custom F450 PX4 SITL configuration with Gazebo.
