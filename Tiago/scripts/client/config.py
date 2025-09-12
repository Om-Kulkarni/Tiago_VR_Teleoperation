# config.py

## --- Network Configuration ---

# IP Address and Port for the real Tiago robot running 'tiago_host.py'
ROBOT_IP = "10.68.0.1"
ROBOT_PORT = 65432

# IP Address and Port for the local server that Unity will connect to
LOCAL_HOST = "127.0.0.1"  # This should almost always be localhost
LOCAL_PORT = 10500

## --- Mode Configuration ---

# Master flag to switch between simulation and real robot control
# True: The bridge will try to connect to the ROBOT_IP
# False: The bridge will run in simulation mode and send data back to Unity
CONNECT_TO_ROBOT = False

## --- Robot and IK Configuration ---

# You MUST update this path to point to your URDF file on your local machine
URDF_FILE_PATH = "Tiago/urdf/tiago_pal_gripper.urdf"

# The name of the link that you want to control as the end-effector
# For Tiago's arm, this is often 'arm_tool_link' or 'gripper_grasping_frame'
END_EFFECTOR_LINK = "arm_tool_link"