# ============================================================
# chess_operations.py
# Purpose: Create easy to use chess functions for taking, or moving pieces, as well as interfacing with Stock-fish

class ChessOrchestrator:

    def __init__(self):
        pass

    def move(self, start_location, end_location):
        """
        Summary: Move chess piece from start_location to empty tile at end_location.

        :param start_location: X-Y coordinate of the piece's starting location.
        :param end_location: X-Y coordinate of empty tile, where piece must be placed.
        :return: NA
        """
        pass

    def remove(self, start_location):
        """
        Summary: (Helper) Take enemy chess pice from start_location to next spot in 'grave-yard' and add to grave-yard queue.

        :param start_location: X-Y coordinate of the piece's starting location.
        :return: NA
        """
        pass

    def take(self, start_location, end_location):
        """
        Summary: Use 'remove()' and 'move()' to complete a full chess move where one piece captures another.

        :param start_location: X-Y coordinate of the attacking piece's starting location.
        :param end_location: X-Y coordinate of captured piece.
        :return:
        """

        self.remove(end_location)
        self.move(start_location, end_location)
        pass

    def promote(self, start_location, end_location, promotion_type):
        """
        Summary: Use 'remove' to take the pawn off of the start_location, then pull desired piece from 'grave-yard' and place on
                 end_location in the back row, making sure to replace piece in grave-yard queue.

        :param start_location: X-Y coordinate of the pawn.
        :param end_location: X-Y coordinate of the promoted piece.
        :param promotion_type: The type of piece used to replace the pawn.
        :return: NA
        """
        pass

    def castle(self, direction):
        """
        Summary: Depending on the 'direction' use the move function to move the King and corresponding Rook to "Castle".

        :param direction: String 'left' or 'right' represents the direction of the castle.
        :return: NA
        """
        pass

    def en_passant(self, start_location, end_location, capture_location):
        """
        Summary: Similar to 'take()' use 'remove' and 'move()' to complete a full chess move where one piece captures another,
                (but the pawn being captured and the final destination of the attacking pawn are not the same).

        :param start_location: Original X-Y coordinate of the attacking pawn.
        :param end_location: New X-Y coordinate of attacking pawn.
        :param capture_location: X-Y coordinate of the captured pawn.
        :return: NA
        """
        pass


# (Testing)  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def main():
    a_chess_orchestrator = ChessOrchestrator()
    print("test 0")

if __name__ == "__main__":
    main()