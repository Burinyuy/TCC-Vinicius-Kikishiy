@[if (ft_sensor_left == "schunk-ft" or ft_sensor_right == "schunk-ft" or end_effector_left == "schunk-wsg" or end_effector_right == "schunk-wsg")]@
force_torque:
@[end if]@
@[if ft_sensor_left == "schunk-ft"]@
  wrist_left_ft:
    sensor_joint: arm_left_7_joint
    frame: wrist_left_ft_link
@[end if]@
@[if end_effector_left == "schunk-wsg"]@
  gripper_left_left_fingertip:
    sensor_joint: gripper_left_left_finger_joint
    frame: gripper_left_left_fingertip_link
  gripper_left_right_fingertip:
    sensor_joint: gripper_left_right_finger_joint
    frame: gripper_left_right_fingertip_link
@[end if]@
@[if ft_sensor_right == "schunk-ft"]@
  wrist_right_ft:
    sensor_joint: arm_right_7_joint
    frame: wrist_right_ft_link
@[end if]@
@[if end_effector_right == "schunk-wsg"]@
  gripper_right_left_fingertip:
    sensor_joint: gripper_right_left_finger_joint
    frame: gripper_right_left_fingertip_link
  gripper_right_right_fingertip:
    sensor_joint: gripper_right_right_finger_joint
    frame: gripper_right_right_fingertip_link
@[end if]@

imu:
  base_imu:
    frame: base_imu_link
    gazebo_sensor_name: imu_sensor

