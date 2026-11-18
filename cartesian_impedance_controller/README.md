# Cartesian Impedance Controller

Vanilla Cartesian pose impedance. This plugin accepts a pose equilibrium and
implements a spring and damper in all six Cartesian directions:

```
tau = J^T (K e - D J q_dot) + model_compensation + nullspace_torque
```

It does not read an FT sensor, accept a target wrench, compensate a sensor load
or remove the spring along a force direction. Those functions belong to the
separate [`force_impedance_controller`](../force_impedance_controller/README.md).
This is a behavior change from the earlier combined controller: migrate any
force-control configuration to the new plugin and its own parameter block.

## Configuration

Merge this into the actual robot configuration, using its joints and frames:

```yaml
controller_manager:
  ros__parameters:
    cartesian_impedance_controller:
      type: cartesian_impedance_controller/CartesianImpedanceController

cartesian_impedance_controller:
  ros__parameters:
    robot_base_link: base_link
    end_effector_link: tool0
    joints: [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
    command_interfaces: [effort]
    state_interfaces: [position, velocity]
    command_current_configuration: false # true for the corresponding KUKA interface
    compensate_gravity: false
    compensate_coriolis: false
    compensate_dJdq: false
    stiffness:
      trans_x: 500.0
      trans_y: 500.0
      trans_z: 500.0
      rot_x: 50.0
      rot_y: 50.0
      rot_z: 50.0
    max_impedance_force: 10.0
    max_impedance_torque: 2.0
    delta_tau_max: 1.0
    task_inertia_cutoff: 0.000001
    nullspace_stiffness: 0.0
    command_timeout_s: 0.0
```

Publish `geometry_msgs/PoseStamped` to
`/<controller_name>/target_frame`, with `header.frame_id == robot_base_link`.
The equilibrium starts at the measured activation pose. Pre-activation messages
are ignored. Stiffness axes follow the current end-effector frame.

`command_timeout_s: 0.0` keeps the last equilibrium indefinitely. A positive
value checks both receive age and ROS timestamp. Missing or expired pose
commands remove the spring and postural task, retaining damping and configured
model compensation. A fresh valid pose resumes control. No wrench command or
FT measurement is required in either mode.

The damping uses regularized Cartesian inertia. `task_inertia_cutoff` floors
eigenvalues of `J M^-1 J^T` in the existing unscaled linear/angular coordinates;
this protects the calculation but does not restore lost motion directions.
The postural projector uses a truncated pseudoinverse. Optional J-dot
compensation and all spring/damping terms are included in the task force/moment
limits. Joint effort and increment limits are enforced by `effort_controller_base`.
`delta_tau_max` is Nm per update, not Nm/s. Logging is OFF by default.

Both plugins can be installed and loaded, but only one can actively claim the
same robot effort interfaces at a time. Applications that need wrench commands
or measured-force feedback must select the force plugin.

## Verification

Build with `-DBUILD_TESTING=ON -DLOGGING=OFF` and run `colcon test`.
`test/controller_lifecycle_test.cpp` checks lifecycle failures, singular models,
pose-only control without FT, reference expiry and reactivation on synthetic
hardware. The force package also tests discovery and loading of both plugins.
