import pybullet as p
import logging
import config

class IKSolver:
    """
    A class to handle Inverse Kinematics calculations for the Tiago robot,
    with logic adapted from a working pybullet simulation.
    """
    def __init__(self, use_gui=False):
        """Initializes the IKSolver by loading the robot into a pybullet instance."""
        logging.info("Initializing IKSolver with pybullet.")
        try:
            # Step 1: Connect to pybullet in GUI or DIRECT mode.
            self.use_gui = use_gui
            if self.use_gui:
                self.physics_client = p.connect(p.GUI)
                p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=[0.5, 0, 0.7])
                # Create a red sphere to visualize the IK target
                sphere_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[1, 0, 0, 0.8])
                self.target_marker_id = p.createMultiBody(baseVisualShapeIndex=sphere_shape)
            else:
                self.physics_client = p.connect(p.DIRECT)
                self.target_marker_id = None

            # Step 2: Load the URDF file.
            logging.info(f"Loading URDF for IK solver from: {config.URDF_FILE_PATH}")
            self.robot_id = p.loadURDF(config.URDF_FILE_PATH, useFixedBase=True)
            logging.info(f"Successfully loaded robot URDF. Robot ID: {self.robot_id}")

            # Step 3: Discover and map all joints, closely following sim_ik_client.py logic.
            self.end_effector_index = -1
            self.torso_ik_index = None
            self.arm_ik_indices = []
            
            # This list will hold the absolute indices of all movable (non-fixed) joints.
            movable_joint_indices = []
            
            num_joints = p.getNumJoints(self.robot_id)
            for i in range(num_joints):
                joint_info = p.getJointInfo(self.robot_id, i)
                joint_name = joint_info[1].decode('utf-8')
                joint_type = joint_info[2]
                link_name = joint_info[12].decode('utf-8')

                # Find the end effector's index by its link name
                if link_name == config.END_EFFECTOR_LINK:
                    self.end_effector_index = i
                
                # If the joint is not fixed, add its index to our list of movable joints
                if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                    movable_joint_indices.append(i)

            # Now, find the position (ik_index) of our specific arm/torso joints within that movable list
            self.torso_ik_index = movable_joint_indices.index(21) # Torso is joint 21
            
            arm_absolute_indices = [31, 32, 33, 34, 35, 36, 37]
            for arm_joint_id in arm_absolute_indices:
                self.arm_ik_indices.append(movable_joint_indices.index(arm_joint_id))

            if self.end_effector_index == -1:
                raise ValueError(f"Could not find end-effector link index for '{config.END_EFFECTOR_LINK}'")

            logging.info(f"Found end effector index: {self.end_effector_index}")
            logging.info(f"Torso IK solution index: {self.torso_ik_index}")
            logging.info(f"Arm IK solution indices: {self.arm_ik_indices}")

            # Store the absolute indices of all joints we are controlling for visualization
            self.controlled_joint_indices = [21] + arm_absolute_indices

        except Exception:
            logging.exception("Failed to initialize pybullet IKSolver")
            self.disconnect()
            raise

    def calculate_ik(self, current_joint_angles, target_position, target_orientation_quaternion):
        """
        Calculates the target joint angles for a given end-effector pose.
        """
        try:
            if self.use_gui and self.target_marker_id is not None:
                p.resetBasePositionAndOrientation(self.target_marker_id, target_position, [0, 0, 0, 1])

            # This call returns a solution for ALL movable joints.
            full_ik_solution = p.calculateInverseKinematics(
                bodyUniqueId=self.robot_id,
                endEffectorLinkIndex=self.end_effector_index,
                targetPosition=target_position,
                targetOrientation=target_orientation_quaternion,
                maxNumIterations=100,
                residualThreshold=1e-4,
                # Using current state as rest poses can help find a stable solution
                restPoses=current_joint_angles
            )
            
            # Extract the specific joint values we care about using our pre-computed indices.
            torso_pose = full_ik_solution[self.torso_ik_index]
            arm_poses = [full_ik_solution[i] for i in self.arm_ik_indices]
            
            # Combine the 8-joint solution
            target_joint_angles = [torso_pose] + arm_poses

            # If in GUI mode, update the robot's pose to visualize the IK solution
            if self.use_gui:
                for i, joint_index in enumerate(self.controlled_joint_indices):
                    p.resetJointState(
                        bodyUniqueId=self.robot_id,
                        jointIndex=joint_index,
                        targetValue=target_joint_angles[i]
                    )
            
            return target_joint_angles

        except Exception:
            logging.exception("IK calculation failed")
            # Return the current angles as a fallback
            return current_joint_angles

    def disconnect(self):
        """Disconnects from the pybullet physics server."""
        if hasattr(self, 'physics_client') and p.isConnected(self.physics_client):
            p.disconnect(self.physics_client)
            logging.info("Disconnected from pybullet.")

# --- Standalone Test ---
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    print("--- Attempting to initialize IKSolver with pybullet to test URDF loading ---")
    
    solver = None
    try:
        solver = IKSolver(False)
        print("\n--- IKSolver initialized successfully with pybullet! ---")
        
        # Example calculation
        print("\n--- Performing test IK calculation ---")
        # A sample starting pose for the 8 controllable joints
        start_angles = [0.15, 1.6, -0.9, -3.1, 1.8, -1.5, -0.6, -1.5] 
        target_pos = [0.5, -0.2, 0.8]
        target_orn = p.getQuaternionFromEuler([0, -1.57, 0])
        
        print(f"Target position: {target_pos}")
        
        joint_angles = solver.calculate_ik(start_angles, target_pos, target_orn)
        
        print(f"Calculated joint angles: {[f'{angle:.3f}' for angle in joint_angles]}")
        assert len(joint_angles) == 8, "Mismatch: Expected 8 joint angles!"
        print("Test calculation successful.")
        
    except Exception as e:
        print(f"\nAn error occurred during initialization or test: {e}")
    finally:
        if solver:
            solver.disconnect()

