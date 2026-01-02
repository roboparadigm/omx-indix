import os

# Define the package name
package_name = 'roboticarm_description'

# Define the executable name
executable_name = 'robotic_arm_urdf_xacro'

# Launch the ROS2 node
os.system('ros2 launch {0} empty_world.launch.py'.format(package_name))