# PX4 Autopilot Firmware for Multirotors with Horizontal Thrust-Vectored Propellers

This repository contains a modified version of PX4-Autopilot firmware (based on release v1.14.2), extending its capabilities to support horizontal propellers (thrusters) with thrust vectoring configurations.

## Features
- **Thrust Vectoring Multirotor**: New vehicle class for multirotors with horizontal thrust-vectored propellers.
- **Geometric SE(3) Position Control**: Advanced position control using geometric formulations.
- **Custom Flight Modes**: Support for tilted attitude control, dual-thruster operation, and auto-thruster mode.

## Setup Instructions
### 1. Set Up Your Development Environment
Follow the official PX4 documentation to set up the development environment:
[PX4 Developer Setup](https://docs.px4.io/main/en/dev_setup/dev_env.html)

### 2. Test the Official Firmware & Simulations
Ensure your setup works correctly by running standard PX4 firmware and SITL simulations.

### 3. Clone the Custom Firmware
Clone this repository, including all submodules:
```sh
git clone --recursive https://github.com/rjros/Horizontal_thrusters.git
```

## Simulation Vehicles
The following vehicles are configured for SITL simulations, each equipped with horizontal thrusters.

### **Base Multicopter Vechicles**
| Vehicle Name | Command |
|-------------|---------|
| Quadrotor | `make px4_sitl gz_custom_x500` |
| Quadrotor with 2 Horizontal Thrusters | `make px4_sitl gz_custom_x500_dual` |

### Thrust Vectoring Using Thrusters at Horizontal Offsets

| Vehicle Horizontal Offset(mm) | Command |
|-------------|---------|
| 200 | make px4_sitl gz__x500_dual_200 |
| 300 | make px4_sitl gz__x500_dual_300|
| 400 | make px4_sitl gz__x500_dual_400|
| 500 | make px4_sitl gz__x500_dual_500|

### **Thrusters at Vertical Offsets**
| Vehicle Vertical Offset (mm) | Command |
|----------------------|-----------------------------|
| 20  | `make px4_sitl gz_custom_x500_20`  |
| -20 | `make px4_sitl gz_custom_x500_20_N` |
| 40  | `make px4_sitl gz_custom_x500_40`  |
| -40 | `make px4_sitl gz_custom_x500_40_N` |
| 60  | `make px4_sitl gz_custom_x500_60`  |
| -60 | `make px4_sitl gz_custom_x500_60_N` |
| 80  | `make px4_sitl gz_custom_x500_80`  |
| -80 | `make px4_sitl gz_custom_x500_80_N` |
| 100 | `make px4_sitl gz_custom_x500_100` |
| -100 | `make px4_sitl gz_custom_x500_100_N` |

## Parameter Configuration
The following parameters define the behavior of the vehicle and its thrust-vectoring capabilities. These can be set in **QGroundControl (QGC)** or manually configured in the initialization files located at:
`/Horizontal_thrusters/ROMFS/px4fmu_common/init.d-posix/airframes/[custom_vehicle]`
### **General Parameters**
| Parameter | Description |
|--------------|------------|
| **CA_AIRFRAME** | Vehicle class **11** or **Thrust Vectoring Multirotor**. |
| **CA_INDEX** | Index of the **first horizontal thruster**. Starts with **vertical propellers first**, followed by **horizontal thrusters**. |
| **RC_SIM** | Control source for the vehicle and thrust vectoring setpoints: **QGC**, **Onboard PC (ROS2)**, or **RC (Remote Controller)**. |
| **TOTAL_MASS** | Total weight of the UAV (**kg**). |
| **VECT_ATT_MODE** | Flight mode for thrust vectoring: **Tilted Attitude**, **Dual-Thrusters**, **Auto-Thruster Mode**. |
| **X_THRUST_MAX** | Max horizontal thrust along **X-axis** (**N**). |
| **Y_THRUST_MAX** | Max horizontal thrust along **Y-axis** (**N**). |
| **Z_THRUST_MAX** | Max vertical thrust (**N**). |
| **X_TORQUE_MAX** | Maximum torque about the **X-axis**, expressed in **N·m**. |
| **Y_TORQUE_MAX** | Maximum torque about the **Y-axis**, expressed in **N·m**. |
| **Z_TORQUE_MAX** | Maximum torque about the **Z-axis**, expressed in **N·m**. |
| **VECT_ANG_N** | Rotation angle for **thruster N**, used only when **RC_SIM = QGC**. |
| **MC_IXX** | **Moment of inertia** about **X-axis** (**kg·m²**). Similar parameters: **MC_IXY, MC_IXZ, MC_IYY, MC_IYZ, MC_IZZ**. |

### **Geometric Position Controller Gains**
| Parameter | Description |
|--------------|------------|
| **GEOM_X_P** | Proportional gain for **position control** along **X-axis**. Similar for **Y, Z** (**GEOM_Y_P, GEOM_Z_P**). |
| **GEOM_X_I** | Integral gain for **position control** along **X-axis**. Similar for **Y, Z** (**GEOM_Y_I, GEOM_Z_I**). |
| **GEOM_X_D** | Derivative gain for **position control** along **X-axis**. Similar for **Y, Z** (**GEOM_Y_D, GEOM_Z_D**). |

### **Thrust Vectoring Position Controller Gains**
| Parameter | Description |
|--------------|------------|
| **GEOM_THX_P** | Proportional gain for **thruster position control** along **X-axis**. Similar for **Y, Z** (**GEOM_THY_P, GEOM_THZ_P**). |
| **GEOM_THX_I** | Integral gain for **thruster position control** along **X-axis**. Similar for **Y, Z** (**GEOM_THY_I, GEOM_THZ_I**). |
| **GEOM_THX_D** | Derivative gain for **thruster position control** along **X-axis**. Similar for **Y, Z** (**GEOM_THY_D, GEOM_THZ_D**). |

### **Attitude and Rate Controller Gains**
| Parameter | Description |
|--------------|------------|
| **GEOM_ROLL_P** | Proportional gain for **attitude control** (roll). Similar for **pitch, yaw** (**GEOM_PITCH_P, GEOM_YAW_P**). |
| **GEOM_ROLLR_P** | Proportional gain for **rate control** (roll rate). Similar for **pitch, yaw** (**GEOM_PITCHR_P, GEOM_YAWR_P**). |
| **GEOM_ROLLR_I** | Integral gain for **rate control** (roll rate). Similar for **pitch, yaw** (**GEOM_PITCHR_I, GEOM_YAWR_I**). |
| **GEOM_ROLLR_D** | Derivative gain for **rate control** (roll rate). Similar for **pitch, yaw** (**GEOM_PITCHR_D, GEOM_YAWR_D**). |

## Citation
If you use this work in an academic context, please cite:
```bibtex
@article{Martinez2025,
  author = {Martinez, R.R. and Paul, H. and Shimonomura, K.},
  title = {Design and Control Strategies of Multirotors with Horizontal Thrust-Vectored Propellers},
  journal = {Drones},
  year = {2025},
  volume = {9},
  pages = {145},
  doi = {10.3390/drones9020145}
}
```
