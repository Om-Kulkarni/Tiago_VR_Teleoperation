# tiago_unity_bridge.py
import socket
import pickle
import struct
import logging

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
            return True # Return True on success
        except Exception as e:
            logging.error(f"Failed to connect to Tiago robot: {e}")
            return False # Return False on failure
        
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
    



    