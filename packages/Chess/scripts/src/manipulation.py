"""
Manipulation wrapper: placeholders for IK repo. Will expose move / pose control.

Once the IK repo is available, install it (e.g.):
  pip install "git+https://github.com/YOUR_ORG/IK_REPO.git@branch#subdirectory=path"

Then uncomment the imports below and implement the methods.
"""

import time

import serial

# Placeholder: uncomment when IK package is available.
# from ik_package.solver import Solver
# from ik_package.robot import Robot


class Manipulation:
    """Thin wrapper around IK repo: move arm to position, etc."""

    CAPTURE_ZONE = "XX"
    LEFT_LIMIT = 'c'
    RIGHT_LIMIT = 'f'
    LEFT_ARM = "LEFT"
    RIGHT_ARM = "RIGHT"
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
    
    def now(self, command: str) -> None:
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
    def get_side(self, move) -> str:
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

        for move in move_queue: #Loop through the move queue 
            pick_up_square = move[0:2]
            side = self.get_side(move)
            self.move_to_square(pick_up_square)
            self.pick_up(side)
            destination_square = move[2:4]
            if destination_square == Manipulation.CAPTURE_ZONE:
                self.move_captured_piece()
            else:
                self.move_to_square(destination_square, side)
        
        self.press_clock()

    def press_clock(self) -> None:
        raise NotImplementedError





def main() -> None:
    Manipulation().home()
    print("home: received Complete")


if __name__ == "__main__":
    main()
