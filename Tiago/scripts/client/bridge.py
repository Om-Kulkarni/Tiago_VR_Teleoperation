import socket
import json
import struct
import logging
from threading import Thread

# Import our custom modules
import config
from ik_solver import IKSolver
from tiago_client import TiagoClient

# A helper list to define the order of joints for communication
# This must match the order in Unity's TeleopController.cs
JOINT_ORDER = [
    'torso_lift_joint', 'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
    'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
]

def handle_unity_connection(unity_conn, ik_solver, robot_client):
    """
    This function runs in a dedicated thread for each connected Unity client.
    It receives a target pose, calculates IK, and then either sends the
    result to the real robot or back to Unity for simulation.
    """
    logging.info("Unity client connected. Starting communication loop.")

    # This will store the latest state received from the real robot
    latest_observation = None

    try:
        while True:
            # Step 1: Receive data from Unity (JSON format)
            raw_msglen = unity_conn.recv(4)
            if not raw_msglen: break
            msglen = struct.unpack('>I', raw_msglen)[0]
            json_data = unity_conn.recv(msglen)
            if not json_data: break
            
            # The message from Unity should contain the target pose and current joint angles
            unity_data = json.loads(json_data.decode('utf-8'))

            # Extract the dictionaries from the payload
            pos_dict = unity_data.get("target_position", {})
            orn_dict = unity_data.get("target_orientation", {})

            # Convert dictionaries into lists that pybullet can understand
            target_pos = [pos_dict.get('x', 0), pos_dict.get('y', 0), pos_dict.get('z', 0)]
            target_orn = [orn_dict.get('x', 0), orn_dict.get('y', 0), orn_dict.get('z', 0), orn_dict.get('w', 1)]

            response_data = {}
            
            
            # --- Step 2: Decide what to do based on the config flag ---
            if config.CONNECT_TO_ROBOT:
                # --- REAL ROBOT MODE ---
                
                # 2a. Determine the current joint angles to use for the IK calculation
                if latest_observation is None:
                    # If we don't have a latest observation, request one from the robot
                    # On the first loop, we have no robot data yet.
                    # Use the angles from Unity as the initial seed.
                    ik_input_angles = unity_data.get("current_joint_angles")
                    logging.info("Using initial joint angles from Unity for first IK calculation.")
                else:
                    # For all subsequent loops, use the real angles from the robot
                    ik_input_angles = [latest_observation.get(f"{name}.pos", 0.0) for name in JOINT_ORDER]

                # 2b. Calculate IK
                target_joint_angles = ik_solver.calculate_ik(ik_input_angles, target_pos, target_orn)

                # 2c. Construct the action dictionary to send to the robot
                action = {
                    'torso_lift_joint.pos': target_joint_angles[0],
                    'arm_joint_positions': target_joint_angles[1:],
                    'base_linear_velocity': unity_data.get("base_linear_velocity", 0.0),
                    'base_angular_velocity': unity_data.get("base_angular_velocity", 0.0),
                    'gripper_command': unity_data.get("gripper_command", 0)
                }

                # 2d. Send the action to the robot and receive the latest observation
                latest_observation = robot_client.send_action_and_get_observation(action)

                # 2e. Format the response data to send back to Unity
                observed_joint_angles = [latest_observation.get(f"{name}.pos", 0.0) for name in JOINT_ORDER]

                response_data = {
                    "status": "success",
                    "joint_trajectory": observed_joint_angles,
                    "gripper_command": action['gripper_command']
                }

            else:
                # --- SIMULATION MODE ---
                current_angles = unity_data.get("current_joint_angles")
                target_joint_angles = ik_solver.calculate_ik(current_angles, target_pos, target_orn)
                response_data = {
                    "status": "success",
                    "joint_trajectory": target_joint_angles,
                    "gripper_command": unity_data.get("gripper_command", 0)
                }

            # Step 3: Send the appropriate response back to Unity
            response_json = json.dumps(response_data).encode('utf-8')
            header = struct.pack('>I', len(response_json))
            unity_conn.sendall(header + response_json)

    except (ConnectionResetError, BrokenPipeError):
        logging.warning("Unity client disconnected.")
    except Exception:
        logging.exception("An error occurred in the communication loop")
    finally:
        logging.info("Closing connection with Unity client.")
        unity_conn.close()

def main():
    """
    Initializes the IK solver and robot client, then starts the server to listen for Unity.
    """
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    ik_solver = None
    robot_client = None
    server_socket = None

    try:
        # Initialize the IK solver
        ik_solver = IKSolver(True)

        # Initialize the real robot client if the flag is set
        if config.CONNECT_TO_ROBOT:
            robot_client = TiagoClient(config.ROBOT_IP, config.ROBOT_PORT)
            if not robot_client.connect():
                raise ConnectionError("Could not connect to the real robot.")
        else:
            logging.info("Running in SIMULATION mode. Will not connect to the real robot.")
            
        # Start the local server to listen for Unity
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # *** KEY CHANGE: Set a timeout on the socket ***
        # This makes the accept() call non-blocking.
        server_socket.settimeout(1.0) 
        
        server_socket.bind((config.LOCAL_HOST, config.LOCAL_PORT))
        server_socket.listen(1)
        logging.info(f"Bridge is listening for Unity on {config.LOCAL_HOST}:{config.LOCAL_PORT}")

        while True:
            try:
                # This will now time out after 1 second if no client connects
                conn, addr = server_socket.accept()
                thread = Thread(target=handle_unity_connection, args=(conn, ik_solver, robot_client))
                thread.daemon = True
                thread.start()
            except socket.timeout:
                # This is expected. Just continue the loop to check for KeyboardInterrupt.
                continue

    except KeyboardInterrupt:
        logging.info("Ctrl+C received. Shutting down the bridge server.")
    except Exception as e:
        logging.error(f"An unexpected error occurred in main: {e}")
    finally:
        # Clean up all resources
        logging.info("Cleaning up resources...")
        if robot_client:
            robot_client.disconnect()
        if ik_solver:
            ik_solver.disconnect()
        if server_socket:
            server_socket.close()
        logging.info("Shutdown complete.")

if __name__ == "__main__":
    main()

