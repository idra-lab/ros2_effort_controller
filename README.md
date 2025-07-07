# Kuka Cartesian Impedance Controller
![build badge](https://github.com/lucabeber/effort_controller/actions/workflows/humble.yml/badge.svg)
![build badge](https://github.com/lucabeber/effort_controller/actions/workflows/jazzy.yml/badge.svg)
![build badge](https://github.com/lucabeber/effort_controller/actions/workflows/rolling.yml/badge.svg)


This branch implements command the joint configuration to the [lbr_stack](https://github.com/idra-lab/lbr_fri_ros2_stack) KUKA hardware interface so it allows `Cartesian Impedance Control` as well as `Joint Position Control` depending on the command input chosen as `client_command_mode` in the hardware interface.

This controller is designed to be used with the KUKA FRI in `POSITION` command mode. To switch FRI Command Mode you need to update the following line in the `LBRServer.java` application, chosing between the two controller.
```
% for cartesian impedance control
control_mode_ = new CartesianImpedanceControlMode(0, 0, 0, 0, 0, 0, 0);
```
```
% for cartesian impedance control
control_mode_ = new PositionControlMode();
```

|Supported Controller Options| FRI Command Mode   | FRI Controller Mode            |
|----------------------------|--------------------|--------------------------------|
|Cartesian Impedance Control |`POSITION`          | `CartesianImpedanceControlMode`|
|Kinematics control          |`POSITION`          | `PositionControlMode`          |

Check out how to use this controller in our KUKA LBR example [here](https://github.com/idra-lab/kuka_impedance)!  
The structure of the code and some libraries have been taken from the repo [Cartesian Controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers).


#### Parameters:
Below is an example `controller_manager.yaml` for a controller specific configuration.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500  # Hz

    kuka_cartesian_impedance_controller:
      type: kuka_cartesian_impedance_controller/KukaCartesianImpedanceController

    # More controller instances here
    # ...

/**/kuka_cartesian_impedance_controller:
  ros__parameters:
    max_linear_velocity: 0.3
    max_angular_velocity: 0.3
    end_effector_link: "lbr_link_ee"
    robot_base_link: "lbr_link_0"
    compliance_ref_link: "lbr_link_ee"
    command_interfaces:
      - effort
    state_interfaces:
      - position
      - velocity
    joints:
      - lbr_A1
      - lbr_A2
      - lbr_A3
      - lbr_A4
      - lbr_A5
      - lbr_A6
      - lbr_A7
    
    # More controllers here
```
#### Nullspace target configuration
The target nullspace configuration can be set in `kuka_cartesian_impedance_controller.cpp` by modifying the `m_q_ns` array. The values are in radians and correspond to the desired joint angles for the nullspace configuration.:
```
  m_q_ns(0) = deg2rad(0.0);
  m_q_ns(1) = deg2rad(40.0);
  m_q_ns(2) = deg2rad(0.0);
  m_q_ns(3) = deg2rad(-72.6);
  m_q_ns(4) = deg2rad(0.0);
  m_q_ns(5) = deg2rad(80.05);
  m_q_ns(6) = deg2rad(0.0);
```