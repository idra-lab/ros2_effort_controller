# ROS 2 Effort Controllers

Robot-independent torque controllers built on `ros2_control`. A shared base
handles robot-model loading, joint state/command interfaces and effort limits;
separate plugins implement Cartesian impedance, force-feedback impedance,
joint impedance and gravity compensation.

[![Humble CI](https://github.com/idra-lab/ros2_effort_controller/actions/workflows/humble.yml/badge.svg)](https://github.com/idra-lab/ros2_effort_controller/actions/workflows/humble.yml)
[![Jazzy CI](https://github.com/idra-lab/ros2_effort_controller/actions/workflows/jazzy.yml/badge.svg)](https://github.com/idra-lab/ros2_effort_controller/actions/workflows/jazzy.yml)
[![Rolling CI](https://github.com/idra-lab/ros2_effort_controller/actions/workflows/rolling.yml/badge.svg)](https://github.com/idra-lab/ros2_effort_controller/actions/workflows/rolling.yml)

The badges describe the existing distribution workflows. The controller split
and synthetic-interface checks described below were tested locally with Humble.

## Choose a controller

| Plugin type | Task | Required commands and feedback |
| --- | --- | --- |
| [`cartesian_impedance_controller/CartesianImpedanceController`](cartesian_impedance_controller/README.md) | Vanilla Cartesian spring-damper impedance in all six directions | Pose equilibrium; joint position and velocity |
| [`force_impedance_controller/ForceImpedanceController`](force_impedance_controller/README.md) | Cartesian impedance with calibrated force/torque feedback; optional hybrid normal-force control | Pose and wrench references; wrist FT stream; joint position and velocity |
| `joint_impedance_controller/JointImpedanceController` | Joint-space impedance | Joint-space references and states |
| `gravity_compensation/GravityCompensation` | Model-based joint gravity compensation | Joint states and robot model |

The Cartesian controllers are distinct libraries and plugin classes. The
vanilla controller does not subscribe to an FT sensor or a target wrench.
The force controller owns sensor compensation and force feedback. Both can be
loaded, but only one controller may actively claim the same effort interfaces.

For KUKA's proprietary position-command controllers, use the existing
[`kuka-prop-ctrl` branch](https://github.com/idra-lab/ros2_effort_controller/tree/kuka-prop-ctrl).
An integration example is available in
[`kuka_impedance`](https://github.com/idra-lab/kuka_impedance).

## Build

Use a ROS distribution with matching system and Python dependencies, source
its environment, and clone this branch into a ROS workspace:

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/effort_ws/src
cd ~/effort_ws/src
git clone --branch feat/force-impedance-controller https://github.com/idra-lab/ros2_effort_controller.git
cd ~/effort_ws
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -r -y
colcon build --packages-up-to cartesian_impedance_controller force_impedance_controller joint_impedance_controller gravity_compensation --cmake-args -DBUILD_TESTING=ON -DLOGGING=OFF
source install/setup.bash
```

Logging through MatLogger2 is optional and OFF by default. Rebuild all derived
controllers when changing `effort_controller_base`.

## Configure the robot

These packages provide controller plugins; the robot driver and
`controller_manager` must be launched by the robot integration. Merge the
selected package's YAML example into that integration's controller config.

For either Cartesian controller, supply:

- `robot_base_link` and `end_effector_link` from the robot's actual model;
- `joints` in actuated KDL chain order, with valid URDF effort limits;
- an `effort` command interface and `position`/`velocity` state interfaces;
- the robot description, either as the controller's `robot_description`
  parameter or on the namespaced `robot_description` topic. Topic waiting is
  bounded by `robot_description_timeout_s` (default 10 s).

Keep gravity and Coriolis compensation consistent with the driver. Set
`command_current_configuration: true` only for the matching KUKA interface;
it additionally claims position command interfaces.

Register both plugin types if switching between them:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500
    cartesian_impedance_controller:
      type: cartesian_impedance_controller/CartesianImpedanceController
    force_impedance_controller:
      type: force_impedance_controller/ForceImpedanceController
```

Each instance needs its own `<controller_name>.ros__parameters` block. Changing
the plugin name does not copy parameters from another controller instance.
The [vanilla configuration](cartesian_impedance_controller/README.md) and
[force configuration](force_impedance_controller/README.md) show those blocks.

After loading and configuring the desired controllers, switch through the
manager rather than activating both on the same joints. For example, with the
vanilla controller active and the force controller already configured/inactive:

```bash
ros2 control switch_controllers --deactivate cartesian_impedance_controller --activate force_impedance_controller --strict
ros2 control list_controllers
```

## Cartesian command contract

| Topic | Type | Consumer |
| --- | --- | --- |
| `/<controller_name>/target_frame` | `geometry_msgs/msg/PoseStamped` | Both Cartesian plugins |
| `/<controller_name>/target_wrench` | `geometry_msgs/msg/WrenchStamped` | Force plugin only |
| Configured `ft_sensor_topic` | `geometry_msgs/msg/WrenchStamped`, sensor-data QoS | Force plugin only |

Pose commands use `robot_base_link`. Both controllers start with the measured
pose as their equilibrium and ignore references received before activation.
For the force plugin, the measured wrench is environment-on-tool; the target
wrench is tool-on-environment. Target wrench and sensor origins have an
explicit contract in the [force README](force_impedance_controller/README.md).

A positive `command_timeout_s` checks receive age and message timestamps.
Publish current timestamps in the same ROS clock domain and refresh references
before the timeout. Timeout zero disables command expiry; it does not disable
the force plugin's sensor freshness and overload checks.

## Migration from the combined Cartesian controller

Move configurations that use FT feedback or wrench targets to
`force_impedance_controller/ForceImpedanceController`. Copy the hardware
parameters into the new instance's parameter block and update command topic
names. FT frame/topic, mass, COM, bias, tare, force gain and hybrid mode belong
to the force plugin. The vanilla plugin now implements pose impedance only.

## Numerical behavior and verification

Both Cartesian plugins use a regularized Cartesian inertia for modal damping
and a truncated pseudoinverse for the optional postural projector. This avoids
inverting a singular matrix directly; it does not recover motion directions
lost at a kinematic singularity. Task force/moment limits and URDF joint effort
limits apply. `delta_tau_max` limits Nm per update, rather than Nm per second.

Stale references inhibit the spring and postural task while retaining damping
and configured model compensation. Force overload is latched until
reactivation. Deactivation writes zero effort directly before releasing the
interfaces; this operation does not itself implement braking or gravity hold.
Dynamic allocations and scheduling still need evaluation at the hardware's
update rate.

```bash
colcon test --packages-select effort_controller_base cartesian_impedance_controller force_impedance_controller
colcon test-result --verbose
```

The synthetic test suites cover 68 shared mathematical checks, 35 vanilla
lifecycle/pose checks, 29 force lifecycle/calibration checks, and discovery and
loading of both plugins in one process. They do not validate a physical robot
or its driver. See the test sources in the corresponding packages.

## Attribution and citation

Parts of the structure and supporting code originate from
[Cartesian Controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers).
If you use this repository in research, please cite:

```bibtex
@article{nardi2026anatomy,
  author={Nardi, Davide and Lamon, Edoardo and Fontanelli, Daniele and Saveriano, Matteo and Palopoli, Luigi},
  journal={IEEE Robotics and Automation Letters}, 
  title={An Anatomy-Aware Shared Control Approach for Assisted Teleoperation of Lung Ultrasound Examinations}, 
  year={2026},
  volume={11},
  number={3},
  pages={2570-2577},
  keywords={Robots;Ribs;Probes;Ultrasonic imaging;Skin;Solid modeling;Computational modeling;Three-dimensional displays;Biological system modeling;Cameras;Medical robots and systems;physical human-robot interaction;telerobotics and teleoperation},
  doi={10.1109/LRA.2026.3653292}}
```
