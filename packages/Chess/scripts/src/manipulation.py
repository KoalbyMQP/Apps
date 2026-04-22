"""
Manipulation wrapper: placeholders for IK repo. Will expose move / pose control.

Once the IK repo is available, install it (e.g.):
  pip install "git+https://github.com/YOUR_ORG/IK_REPO.git@branch#subdirectory=path"

Then uncomment the imports below and implement the methods.
"""

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

    def __init__(self) -> None:
        pass

    def move_to_square(self, square: str, side: str) -> None:
        """Placeholder: move arm to chess square (e.g. 'A1')."""
        raise NotImplementedError("IK repo not installed; add repo and implement.")
    
    def move_captured_piece(self):
        #move the piece to the side
        self.drop("RIGHT")
        raise NotImplementedError


    def home(self) -> None:
        """Placeholder: move arm to home position."""
        raise NotImplementedError("IK repo not installed; add repo and implement.")
    
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
    print("Manipulation: placeholder (IK repo not wired up yet).")
    m = Manipulation()
    print("  Manipulation() OK; move_to_square / home will raise until implemented.")


if __name__ == "__main__":
    main()
