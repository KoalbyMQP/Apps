import zmq
import time
import argparse
import serial  

class ClientPi:
    def __init__(self, host_ip):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{host_ip}:5561")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        self.confirm_sender = self.context.socket(zmq.PUB)
        self.confirm_sender.connect(f"tcp://{host_ip}:5562")

        self.current_gripper = 1  # Default hand on startup
        self.previous_gripper = None  

        # ---  Set up serial connection to Arduino ---
        try:
            # Adjust the device path and baud rate as needed.
            #not sure what the buad rate is here in this example 
            self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1) 
            time.sleep(2)  
            self.arduino.reset_input_buffer()  
            print("Connected to Arduino successfully.")
        except Exception as e:
            print("Error connecting to Arduino:", e)
            self.arduino = None
        # --- END NEW ---

    def handle_gripper_command(self, command):
        try:
            parts = command.split()
            if len(parts) < 4:
                print("Invalid command format")
                return
                
            # Command expected in the format: "<something> <gripper_num> <voice_sent> <host_forwarded>"
            _, gripper_num, voice_sent, host_forwarded = parts
            gripper_num = int(gripper_num)
            
            # Check if already using this gripper
            if gripper_num == self.current_gripper:
                print(f"Already using gripper {gripper_num} - skipping swap")
                self._send_confirmation(
                    current_gripper=self.current_gripper,
                    previous_gripper=self.current_gripper,
                    voice_sent=voice_sent,
                    host_forwarded=float(host_forwarded),
                    client_received=time.time(),
                    processing_time=0,
                    status="already_active"
                )
                return
                
            # Store previous before updating
            self.previous_gripper = self.current_gripper
            self.current_gripper = gripper_num
            
            host_forwarded = float(host_forwarded)
            
            client_received = time.time()
            network_latency = client_received - host_forwarded
            print(f"Host->Client latency: {network_latency*1000:.2f}ms")
            
            # Process command: switch gripper and send command to Arduino
            start_process = time.time()
            self._gripper_switch(gripper_num)
            processing_time = time.time() - start_process
            
            # Send confirmation 
            self._send_confirmation(
                current_gripper=self.current_gripper,
                previous_gripper=self.previous_gripper,
                voice_sent=voice_sent,
                host_forwarded=host_forwarded,
                client_received=client_received,
                processing_time=processing_time
            )
            
        except (ValueError, IndexError):
            
            print("Invalid command format")

    def _gripper_switch(self, num):
        """Switch case for different grippers and send command to Arduino."""
        print(f"\n--- Performing action for gripper {num} ---")
        
        # ---  Send the gripper number to Arduino ---
        if self.arduino is not None:
            # Format the command in a way your Arduino can parse. Here we send:
            #   GRIPPER <num>
            command_str = f"{num}\n"
            try:
                self.arduino.write(command_str.encode())
                print(f"Sent to Arduino: {command_str.strip()}")
                time.sleep(0.1)
                response_lines = []
                start = time.time()
                while time.time() - start < 1:
                    line = self.arduino.readline().decode().strip()
                    if line:
                        response_lines.append(line)
                    else:
                        break
                if response_lines:
                    print("Arduino response:")
                    for line in response_lines:
                        print(line)
                else:
                    print("No response from Arduino")
            except Exception as e:
                print("Error sending command to Arduino:", e)
        else:
            print("Arduino connection not available. Skipping sending command.")
        # --- END NEW ---
        
        # Local action (e.g., logging or additional processing):
        {
            1: lambda: print("Swapping to default hand"),
            2: lambda: print("Swapping to scoop gripper"),
            3: lambda: print("Swapping to vitals gripper"),
            4: lambda: print("Swapping to thermometer gripper"),
            5: lambda: print("Swapping to board game gripper"),
            6: lambda: print("Swapping to main gripper"),
            7: lambda: print("Swapping to type 2 gripper"),
            8: lambda: print("Swapping to type 3 gripper"),
            9: lambda: print("Swapping to type 4 gripper"),
            10: lambda: print("Swapping to type 5 gripper"),
        }.get(num, lambda: print("Invalid gripper number"))()

    def _send_confirmation(self, current_gripper, previous_gripper, voice_sent, 
                           host_forwarded, client_received, processing_time, status="success"):
        message = (f"{current_gripper}|{previous_gripper}|{voice_sent}|{host_forwarded}|"
                   f"{client_received}|{processing_time}|{time.time()}|{status}")
        self.confirm_sender.send_string(message)
        print(f"Sent confirmation: {message}")

    def start_listening(self):
        print("ClientPi started. Waiting for commands...")
        while True:
            try:
                message = self.socket.recv_string(flags=zmq.NOBLOCK)
                self.handle_gripper_command(message)
            except zmq.Again:
                time.sleep(0.1)
            except Exception as e:
                print(f"Error: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Client Pi for gripper control')
    parser.add_argument('--host-ip', 
                       required=False,
                       default='130.215.15.231',
                       help='IP address of the host Pi (default: %(default)s)')
    args = parser.parse_args()
    
    client = ClientPi(args.host_ip)
    client.start_listening()