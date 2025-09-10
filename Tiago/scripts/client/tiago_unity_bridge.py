# tiago_unity_bridge.py
import socket
import json
import pickle
import struct
import logging
from threading import Thread

# --- Configuration ---
ROBOT_IP = "10.68.0.1"  # IP of your Tiago robot
ROBOT_PORT = 65432

LOCAL_HOST = "127.0.0.1"  # IP for Unity to connect to (localhost)
LOCAL_PORT = 10500        # Port for Unity to connect to

# --- Set up basic logging ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class TiagoClient:
    """A simplified client to handle TCP communication with the Tiago robot host."""
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self):
        """Establishes a TCP connection to the Tiago robot host."""
        try:
            logging.info(f"Connecting to Tiago robot at {self.host}:{self.port}...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            logging.info("Connected to Tiago robot.")
        except Exception as e:
            logging.error(f"Failed to connect to Tiago robot: {e}")
            raise False
        
    def disconnect(self):
        """Closes the TCP connection."""
        if self.sock:
            self.sock.close()
            self.sock = None
            logging.info("Disconnected from Tiago robot.")

    def _send_msg(self, msg):
        """Sends a serialized message with a length header."""
        payload = pickle.dumps(msg, protocol=2)  # protocol=2 for Python 2 compatibility
        header = struct.pack('>I', len(payload))
        self.sock.sendall(header + payload)

    def _recv_msg(self):
        """Receives a serialized message with a length header."""
        raw_msglen = self.sock.recv(4)
        if not raw_msglen:
            return None
        msglen = struct.unpack('>I', raw_msglen)[0]

        data = bytearray()
        while len(data) < msglen:
            packet = self.sock.recv(msglen - len(data))
            if not packet:
                return None
            data.extend(packet)
        return pickle.loads(data)
    
    def send_action_and_get_observation(self, action):
        """Sends an action to the robot and retrieves the latest observation."""
        if not self.sock:
            raise ConnectionError("Not connected to Tiago robot.")
        
        self._send_msg(action)
        observation = self._recv_msg()
        return observation
    

def handle_unity_connection(unity_conn, robot_client):
    """Handles communication between Unity and the Tiago robot."""
    logging.info("Unity client connected.")
    try:
        while True:
            # Receive action from Unity (JSON)
            raw_msglen = unity_conn.recv(4)
            if not raw_msglen: break
            msglen = struct.unpack('>I', raw_msglen)[0]

            json_data = unity_conn.recv(msglen)
            if not json_data: break

            action = json.loads(json_data.decode('utf-8'))

            # Send action to Tiago and get observation (Pickle)
            observation = robot_client.send_action_and_get_observation(action)
            if observation is None: break

            # Send observation back to Unity (JSON)
            json_obs = json.dumps(observation).encode('utf-8')
            header = struct.pack('>I', len(json_obs))
            unity_conn.sendall(header + json_obs)

    except (ConnectionResetError, BrokenPipeError):
        logging.warning("Unity client disconnected.")

    except Exception as e:
        logging.error(f"Error during Unity communication: {e}")

    finally:
        logging.info("Closing Unity connection.")
        unity_conn.close()


def main():
    """ Initializes the Tiago client and listens for Unity connections. """
    robot_client = TiagoClient(ROBOT_IP, ROBOT_PORT)
    if not robot_client.connect():
        return
    
    # Setup local server to listen for Unity
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((LOCAL_HOST, LOCAL_PORT))
    server_socket.listen(1)
    logging.info(f"Bridge is listening for Unity on {LOCAL_HOST}:{LOCAL_PORT}")

    try:
        while True:
            conn, addr = server_socket.accept()
            # Handle the connection in a new thread.
            thread = Thread(target=handle_unity_connection, args=(conn, robot_client))
            thread.daemon = True
            thread.start()
    except KeyboardInterrupt:
        logging.info("Shutting down the bridge server.")
    finally:
        robot_client.disconnect()
        server_socket.close()

if __name__ == "__main__":
    main()
    