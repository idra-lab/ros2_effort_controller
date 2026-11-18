# Joint Impedance Controller

Joint-space spring-damper impedance with a Cartesian pose command interface.
The desired TCP pose is converted to a joint equilibrium by inverse kinematics;
the joint torque law is:

```
tau = K_joint .* (q_desired - q) - D_joint .* q_dot + model_compensation
```

This plugin does not implement force feedback or accept a target wrench.
For Cartesian impedance with measured-force feedback, use the separate
[`force_impedance_controller`](../force_impedance_controller/README.md).

## Configuration

Merge this into the real robot controller config. Supply a robot description,
the actual chain-ordered joint names and the correct base/TCP frames; see the
[repository README](../README.md#configure-the-robot).

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500
    joint_impedance_controller:
      type: joint_impedance_controller/JointImpedanceController

joint_impedance_controller:
  ros__parameters:
    robot_base_link: base_link
    end_effector_link: tool0
    joints: [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
    command_interfaces: [effort]
    state_interfaces: [position, velocity]
    command_current_configuration: false
    compensate_gravity: false
    compensate_coriolis: false
    delta_tau_max: 1.0
    command_timeout_s: 0.0
    stiffness:
      joint1: 100.0
      joint2: 100.0
      joint3: 100.0
      joint4: 50.0
      joint5: 50.0
      joint6: 50.0
      joint7: 50.0
    # Optional damping overrides, e.g. damping: {joint1: 20.0}
```

`stiffness.jointN` and `damping.jointN` refer to the Nth entry in `joints`, not
to a literal URDF joint name. Parameters are declared during configuration for
the actual number of joints, including chains with more than seven joints.
Missing stiffness defaults to 100.0; damping defaults to `2*sqrt(stiffness)`.
All gains must be finite and nonnegative. Zero gains are permitted.

For revolute joints, stiffness is Nm/rad and damping is Nm s/rad. For prismatic
joints, use N/m and N s/m. The legacy damping formula assumes unit effective
inertia; it does not establish critical damping for an arbitrary coupled robot.
Tune explicit damping values for the hardware. Parameters are read on configure;
there is no live gain-update contract.

## Commands and lifecycle

Publish `geometry_msgs/msg/PoseStamped` to
`/<controller_name>/target_frame`, using `header.frame_id == robot_base_link`.
Positions and quaternion components must be finite. A nonzero quaternion is
normalized; unsupported frames and invalid values are ignored.

Activation records the measured joint configuration as the equilibrium,
including its redundant posture. It ignores buffered pre-activation commands.
A newly received valid pose is solved using the measured configuration as the
IK seed. The joint equilibrium remains fixed between commands; IK is not run
every update to hold the same target. Only the latest buffered pose is consumed.
The callback and update thread exchange snapshots through a realtime buffer.

If IK fails or returns nonfinite joint values, the previous joint spring is
inhibited until another valid pose command solves successfully. The controller
does not keep pulling toward an obsolete equilibrium after an unreachable
command. Damping and configured gravity/Coriolis compensation remain active.

`command_timeout_s: 0.0` keeps an accepted equilibrium indefinitely. Set a
positive timeout to require fresh receive time and a fresh ROS timestamp.
Timestamps more than 50 ms in the future fail that check. Expired references
inhibit the spring until a new fresh, reachable pose arrives. Publishers must
share the controller's ROS clock domain. Rejected messages do not refresh the
watchdog or replace the last accepted command.

Task effort is limited using the URDF limits, including continuous joints,
and `delta_tau_max` limits effort increments per update. Spring inhibition
changes the requested torque; output still follows these effort/increment
limits. Invalid measured states or model results return a controller error and
clear effort commands. Deactivation clears efforts before releasing interfaces;
zero effort alone does not provide braking or gravity support.

## Migration and limitations

The old `target_wrench` subscription stored a wrench that was never used in
the joint control law. It has been removed, together with the unused FT link,
`hand_frame_control`, `nullspace_stiffness` and debug-output scaffolding.
No FT frame or sensor topic is required to configure this plugin.

The command interface remains Cartesian, so it still depends on IK reachability
and kinematic singularities. There is no direct joint-target topic, trajectory
generator or active singularity avoidance. IK runs in the update thread when
a new pose arrives; the solver has a bounded iteration count, but its worst-case
execution time and the remaining allocations need profiling at the hardware
update rate. This is not a hard-real-time timing guarantee.

## Verification

```bash
colcon build --packages-up-to joint_impedance_controller --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select joint_impedance_controller
colcon test-result --verbose
```

`test/joint_controller_lifecycle_test.cpp` runs 72 checks using synthetic joint
interfaces and URDF chains to check 6/7/8-joint configurations, gains, redundant joint posture,
valid/invalid Cartesian messages, IK failure, stale references, reactivation,
nonfinite states and effort limits. It does not exercise a physical robot.
