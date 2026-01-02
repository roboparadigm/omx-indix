(base) root@masternode:~/ros2_ws# colcon build --packages-select st3215_bridge rparm --symlink-install


(base) root@masternode:~/ros2_ws# source /root/ros2_ws/install/setup.bash

Sim: 
ros2 launch rparm rparm_gazebo.launch.py

cd /root/ros2_ws/src/open_manipulator/roboticarm/src/teleop
python3 arm_nudge.py --mode nudge --duration_sec 6 --sim_gripper


Real: 
ros2 launch rparm rparm_bringup.launch.py port_name:=/dev/ttyACM0 servo_speed_steps_per_s:=400 servo_acc_steps_per_s2:=20

cd /root/ros2_ws/src/open_manipulator/roboticarm/src/teleop
python3 arm_nudge.py --mode nudge








***********************************************************************
For the other joints, the same alignment rules apply. To get real and sim to match like OM-X, do this:

Zero alignment before mounting horns: Power each servo, command its true zero/mid (e.g., 180° or tick midpoint), then mount the horn so the link is in the URDF’s zero pose. If the horns were not centered, you’ll see constant offsets.

Verify URDF kinematics: Ensure link lengths/origins and joint axes match the real hardware. A wrong origin/axis will give different end-poses even with the same joint angles.

Limits and units: Make sure hardware limits and URDF limits agree. Clamp in the bridge for joints 1–4 (like you do for joint6) so teleop can’t drive past the physical range. Confirm everything is in radians on the ROS side.

Controller mapping: Confirm joint ordering matches /arm_controller and the bridge (joint1..joint6). Any mismatch will scramble motions.

Gripper model mismatch: In sim it’s prismatic; in real it’s revolute. Either change the sim gripper to a revolute mimic (like OM-X) or add a conversion so joint6 means the same thing in both.

Compare and tune:

Move real arm to a known pose, ros2 topic echo --once /joint_states to capture the angles.
Send those angles in Gazebo; if the pose differs, check URDF zeros/axes and adjust.
If the error is a constant offset per joint, update the horn alignment or apply an offset in URDF/bridge.
End-effector check: Place the tip on a known marker in real, log joint_states, run the same in sim, and compare the end-effector pose (using TF or a simple FK check). Adjust URDF origins/offsets until they match.

If you do the above—center horns, align URDF axes/lengths, match limits, fix the gripper model/units, and clamp joints in the bridge—your arm will behave much closer to OM-X parity between sim and hardware.