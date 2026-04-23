"""
Manipulation wrapper: placeholders for IK repo. Will expose move / pose control.

Once the IK repo is available, install it (e.g.):
  pip install "git+https://github.com/YOUR_ORG/IK_REPO.git@branch#subdirectory=path"

Then uncomment the imports below and implement the methods.
"""

import time
import lead_screw
import serial

# Placeholder: uncomment when IK package is available.
# from ik_package.solver import Solver
# from ik_package.robot import Robot


class Manipulation:
    """Thin wrapper around IK repo: move arm to position, etc."""

    CAPTURE_ZONE = "X9"
    LEFT_LIMIT = 'c'
    RIGHT_LIMIT = 'f'
    LEFT_ARM = "L"
    RIGHT_ARM = "R"
    UART_PORT = "/dev/ttyACM0"
    UART_BAUD = 115200
    UART_LINE_TIMEOUT = 1.0

    def __init__(self) -> None:
        pass

    def move_to_square(self, square: str, side: str) -> None:
        """Placeholder: move arm to chess square (e.g. 'A1')."""
        raise NotImplementedError("IK repo not installed; add repo and implement.")
    
    def move_captured_piece(self):
        #move the piece to the side
        self.drop("RIGHT")
        raise NotImplementedError


    def home(
        self
    ) -> None:
        """Send the home command to the Arduino and wait for completion."""
        self.now("Starting Position")
    
    def send_move(self, command: str) -> None:
        """
        UART to Arduino Mega: send ``command``, then block until a line
        ``Complete`` is received (other lines are ignored).
        """
        with serial.Serial(
            self.UART_PORT,
            self.UART_BAUD,
            timeout=self.UART_LINE_TIMEOUT,
        ) as ser:
            time.sleep(2)
            ser.reset_input_buffer()
            ser.write(f"{command}\n".encode())
            ser.flush()
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                if raw.decode(errors="replace").strip() == "Complete":
                    break
    
    def pick_up(self, side: str) -> None:
        raise NotImplementedError
    
    def drop(self, side: str):
        raise NotImplementedError
    
    #Gets the side the piece was picked up on
    def get_side(self, move: str) -> str:
        file = move[0]
        if file <= ord(Manipulation.LEFT_LIMIT):
            return self.LEFT_ARM
        elif file >= ord(Manipulation.RIGHT_LIMIT):
            return self.RIGHT_ARM
        else:
            target_file = move[2]
            if ord('a') <= target_file <= ord('d'):
                return self.LEFT_ARM
            else:
                return self.RIGHT_ARM
    
    def make_move_list(self, move_queue) -> None:
        new_moves = []

        for move in move_queue: #Loop through the move queue 
            side = self.get_side(move=move)
            move += side
            new_moves.append(move)

        new_moves.append("CLOCK")
        return new_moves

    def move_list_arduino(self, move_list) -> int:
        for move in move_list:
            lead_screw_1 = move[1]
            lead_screw_2 = move[3]
            column_1 = move[0]
            column_2 = move[2]
            side_bit = move[4]
        if move == "CLOCK":
            lead_screw.send_clock_position()

            lead_screw.send_opponent_position()
        else:
            lead_screw.send_move(lead_screw_1)
            move_1 = str(column_1 + side_bit)
            self.send_move(move_1)
            lead_screw.send_move(lead_screw_2)
            move_2 = str(column_2 + side_bit)
            self.send_move(move_2)
        return 1

def main() -> None:
    Manipulation().home()
    print("home: received Complete")


if __name__ == "__main__":
    main()
