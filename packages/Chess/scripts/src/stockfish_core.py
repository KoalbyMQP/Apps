import stockfish
from stockfish import Stockfish
import re

class Stockfish_Core:
    #CONSTANTS
    NUM_SQUARES = 8
    EMPTY_SQUARE_VALUE = 0

    STARTING_FEN: str = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"

    # Regex to pull positional metadate out of FEN
    REGEX_POSITION_PATTERN = r"^([prnbqkPRNBQK1-8]+(?:/[prnbqkPRNBQK1-8]+){7})(?:\s|$)"
    # Regex to pull en passant capture square out of FEN
    REGEX_EN_PASSANT_PATTERN = r"^(\S+)\s+(\S+)\s+(\S+)\s+(\S+)"
    EN_PASSANT_GROUP_NUMBER = 4  # Group number for EP square
    #Initialize Stockfish with the given path
    def __init__(self, stockfish_path: str):
        self.stockfish = Stockfish(stockfish_path)
        self.stockfish.set_fen_position(self.STARTING_FEN)
        self.last_board_array = self.fen_to_arr(self.STARTING_FEN)
        self.current_FEN = self.STARTING_FEN

    def new_game(self):
        print("game reset")
        self.stockfish.set_fen_position(self.STARTING_FEN)

    #Camera will pass in an array of pieces, stockfish will do work behind the scenes to
    def get_move_from_camera(self, board_dict) -> str:
        fen_move = self.dict_to_fen(board_dict)

        previous_fen = self.stockfish.get_fen_position()

        previous_squares = self.fen_to_arr(previous_fen)

        print("FEN: ", fen_move)

        current_squares = self.fen_to_arr(fen_move)

        different_squares = self.compare_arrays(previous_squares, current_squares)

        last_move = self.get_move(different_squares)
        try:
            self.make_move(last_move)
            robot_move = self.make_move()
        except ValueError:
            print("Stockfishy no likey the movey")
            self.stockfish.set_fen_position(previous_fen)
            raise ValueError

        return robot_move

    #Transform standard FEN notation into an array of all of the pieces
    def fen_to_arr(self, fen: str) -> list[list[str]]:
        # Regex pattern match to only take the piece locations from the fen
        fen_pattern = re.compile(self.REGEX_POSITION_PATTERN)

        # Eliminate the other metadata from the FEN
        piece_locations = fen_pattern.match(fen).group(1)

        # Break up the position rank by rank
        ranks = piece_locations.split("/")

        # Initialize an empty 8x8 array
        board = [[self.EMPTY_SQUARE_VALUE for row in range(self.NUM_SQUARES)] for col in range(self.NUM_SQUARES)]
        # Iterate through the rows, and update the board
        rank_num = 0
        for rank in ranks:
            col = 0
            for ch in rank:
                #print(ch)
                if ch.isdigit():
                    col += int(ch)
                else:
                    #print(rank_num, col)
                    board[rank_num][col] = ch
                    col += 1
            rank_num += 1
        return board

    #Get a visual of the current board
    def get_board_visual(self):
        return self.stockfish.get_board_visual()

    #Gets the move from stockfish
    def get_move(self, changed_squares: list[tuple[int, int]]) -> str:
        print("ChangedSquares:", changed_squares)
        
        coordinates = []
        white_pieces = [Stockfish.Piece.WHITE_PAWN, Stockfish.Piece.WHITE_KNIGHT, Stockfish.Piece.WHITE_BISHOP,
                        Stockfish.Piece.WHITE_ROOK, Stockfish.Piece.WHITE_QUEEN, Stockfish.Piece.WHITE_KING]
        for square in changed_squares:
            coordinates.append(self.board_to_piece_coords(square))
        if len(changed_squares) == 4:
            # castling - only 4 cases hard code the different pairs
            if ("e1" in coordinates) and ("g1" in coordinates):
                return "e1g1"
            elif ("e1" in coordinates) and ("c1" in coordinates):
                return "e1c1"
            elif ("e8" in coordinates) and ("g8" in coordinates):
                return "e8g8"
            else:
                return "e8c8"
        elif len(changed_squares) == 3:
            # for three changed squares, only en passant is possible
            print("en passant detected")
            en_passant_square = self.get_en_passant_square(self.stockfish.get_fen_position())
            if coordinates[0] == en_passant_square:
                if self.stockfish.get_what_is_on_square(coordinates[1]) is None:
                    return coordinates[1] + coordinates[2]
                else:
                    return coordinates[2] + coordinates[1]
            elif coordinates[1] == en_passant_square:
                if self.stockfish.get_what_is_on_square(coordinates[0]) is None:
                    return coordinates[0] + coordinates[2]
                else:
                    return coordinates[2] + coordinates[0]
            else:
                if self.stockfish.get_what_is_on_square(coordinates[0]) is None:
                    return coordinates[0] + coordinates[1]
                else:
                    return coordinates[1] + coordinates[0]
        elif len(changed_squares) == 2:
            if self.stockfish.get_what_is_on_square(square=coordinates[0]) is None:
                return coordinates[1] + coordinates[0]
            elif self.stockfish.get_what_is_on_square(square=coordinates[1]) is None:
                return coordinates[0] + coordinates[1]
            else:
                piece_1 = self.stockfish.get_what_is_on_square(square=coordinates[0])
                if piece_1 in white_pieces:
                    return coordinates[1] + coordinates[0]
                else:
                    return coordinates[0] + coordinates[1]
        else:
            print("Illegal move detected")
            raise ValueError
    #Make the move from the current position
    def make_move(self, move: str=None):
        self.stockfish.set_fen_position(self.current_FEN)

        #If no move is provided, stockfish finds the best move and makes it
        if move is None:
            move = self.stockfish.get_best_move()

        if move:

            print(f"Stockfish plays: {move}")
            print(self.stockfish.get_fen_position())
            self.stockfish.make_moves_from_current_position([move])
            self.current_FEN = self.stockfish.get_fen_position()

            return move
        else:
            print("NO VALID MOVES - GAME IS OVER")
            return 0

    #Takes an array and returns the positional FEN from the array.
    # Board cells are empty (0 or "") or "color-piece_name"; they are converted to FEN chars here.
    def arr_to_positional_fen(self, board: list[list[str]]) -> str:
        piece_to_fen = {
            "white-pawn": "P",
            "white-knight": "N",
            "white-bishop": "B",
            "white-rook": "R",
            "white-queen": "Q",
            "white-king": "K",
            "black-pawn": "p",
            "black-knight": "n",
            "black-bishop": "b",
            "black-rook": "r",
            "black-queen": "q",
            "black-king": "k",
        }
        fen = ""
        for row in board:
            curr_length = 0
            for square in row:
                if square == self.EMPTY_SQUARE_VALUE or square == "":
                    curr_length += 1
                else:
                    if curr_length > 0:
                        fen += str(curr_length)
                        curr_length = 0
                    fen += piece_to_fen.get(square, square)
            if curr_length > 0:
                fen += str(curr_length)
            fen += "/"
        return fen[:-1]

    #Compares two arrays and returns a list of squares that are different
    def compare_arrays(self, previous: list[list[str]], current: list[list[str]]) -> list[tuple[int, int]]:
        squares = []

        for row in range(self.NUM_SQUARES):
            for col in range(self.NUM_SQUARES):
                if previous[row][col] != current[row][col]:
                    squares.append((row, col))

        return squares

    #checks if the game is over by seeing if there are no legal moves in the position
    def is_game_over(self):
        moves = self.stockfish.get_best_move()
        if not moves:
            return True
        else:
            return False

    #Finds the en passant square from the FEN
    def get_en_passant_square(self, fen: str):

        #Regex pattern matching
        match = re.match(self.REGEX_EN_PASSANT_PATTERN, fen)

        if match:
            en_passant_square = match.group(self.EN_PASSANT_GROUP_NUMBER)  # location of the regex square
            print("En passant square:", en_passant_square)
            return en_passant_square
        else:
            print("Invalid FEN")
            return 0

    def board_to_piece_coords(self, coords):
        files = ["a", "b", "c", "d", "e", "f", "g", "h"]
        file = files[coords[1]]

        rank = str(self.NUM_SQUARES - coords[0])
        return file + rank

    def dict_to_fen(self, board_dict):
        # FEN piece letter mapping
        fen_map = {
            "pawn": "p",
            "knight": "n",
            "bishop": "b",
            "rook": "r",
            "queen": "q",
            "king": "k"
        }

        ranks = []

        pieces = [[0 for _ in range(8)] for _ in range(8)]

        current_rank = ""
        empty_count = 0
        rank_number = self.NUM_SQUARES

        # Process squares rank by rank (A8 → H8, ..., A1 → H1)
        for i, (square, piece) in enumerate(board_dict.items()):

            file = square[0]
            row = square[1]
            print(("FILE: ", file, "ROW: ", row))

            row_num = 8 - int(row)
            col_num = ord(file) - ord('A')
            print(("ROW: ", row_num, "COL: ", col_num))
            pieces[row_num][col_num] = piece

            if piece == "":  # empty square
                empty_count += 1
            else:
                # flush any pending empty squares
                if empty_count > 0:
                    current_rank += str(empty_count)
                    empty_count = 0

                color, name = piece.split("-")  # "white-bishop" → ["white", "bishop"]
                letter = fen_map[name]

                # uppercase for white, lowercase for black
                if color == "white":
                    letter = letter.upper()

                current_rank += letter

            # At end of rank (every 8 squares)
            if (i + 1) % self.NUM_SQUARES == 0:
                # flush remaining empty count
                if empty_count > 0:
                    current_rank += str(empty_count)
                    empty_count = 0

                ranks.append(current_rank)
                current_rank = ""
                rank_number -= 1

        # Join ranks with '/'
        pos = self.arr_to_positional_fen(pieces)
        
        return pos + " w KQkq - 0 1"


#MAIN
if __name__ == "__main__":
    # Path to Stockfish executable
    STOCKFISH_PATH = "C:\\Users\\Max\\Downloads\\stockfish-windows-x86-64-avx2\\stockfish\\stockfish-windows-x86-64-avx2.exe"

    # Initialize Stockfish core
    core = Stockfish_Core(STOCKFISH_PATH)

    print("Chess system ready. Stockfish moves first.\n")

    # -------- Robot's first move --------
    first_move = core.make_move()
    print(f"Stockfish plays: {first_move}")
    print(core.get_board_visual())
    print("\nWaiting for human move...\n")

    while True:
        # -------- Human Turn --------
        input("Press Enter after making your move on the board...")
        print("SIMULATING ROBOT MOVE")
        # Get the board state camera
        board_dict = {
            'A8': 'black-rook', 'B8': 'black-knight', 'C8': 'black-bishop', 'D8': 'black-queen', 'E8': 'black-king', 'F8': 'black-bishop', 'G8': 'black-knight', 'H8': 'black-rook',
            'A7': 'black-pawn', 'B7': 'black-pawn', 'C7': 'black-pawn', 'D7': 'black-pawn', 'E7': '', 'F7': 'black-pawn', 'G7': 'black-pawn', 'H7': 'black-pawn',
            'A6': '', 'B6': '', 'C6': '', 'D6': '', 'E6': '', 'F6': '', 'G6': '', 'H6': '',
            'A5': '', 'B5': '', 'C5': '', 'D5': '', 'E5': 'black-pawn', 'F5': '', 'G5': '', 'H5': '',
            'A4': '', 'B4': '', 'C4': '', 'D4': '', 'E4': 'white-pawn', 'F4': '', 'G4': '', 'H4': '',
            'A3': '', 'B3': '', 'C3': '', 'D3': '', 'E3': '', 'F3': '', 'G3': '', 'H3': '',
            'A2': 'white-pawn', 'B2': 'white-pawn', 'C2': 'white-pawn', 'D2': 'white-pawn', 'E2': '', 'F2': 'white-pawn', 'G2': 'white-pawn', 'H2': 'white-pawn',
            'A1': 'white-rook', 'B1': 'white-knight', 'C1': 'white-bishop', 'D1': 'white-queen', 'E1': 'white-king', 'F1': 'white-bishop', 'G1': 'white-knight', 'H1': 'white-rook'
        }
        #run the logic to get the move from the dictionary

        # current_fen = input("Enter the current board FEN: ")

        # Update Stockfish core with the new board and calculate the engine's move
        core.get_move_from_camera(board_dict)

        print("ENGINE MOVED")
        print(core.get_board_visual())

        print("\nReady for next human move...\n")

    # board_dict = {
    #     'A8': '', 'B8': '', 'C8': '', 'D8': '', 'E8': '', 'F8': '', 'G8': 'white-bishop', 'H8': '',
    #     'A7': '', 'B7': '', 'C7': '', 'D7': '', 'E7': '', 'F7': '', 'G7': '', 'H7': '',
    #     'A6': '', 'B6': '', 'C6': 'white-bishop', 'D6': 'black-rook', 'E6': '', 'F6': '', 'G6': '', 'H6': '',
    #     'A5': '', 'B5': '', 'C5': '', 'D5': '', 'E5': '', 'F5': 'white-bishop', 'G5': '', 'H5': '',
    #     'A4': '', 'B4': 'white-bishop', 'C4': '', 'D4': '', 'E4': '', 'F4': '', 'G4': '', 'H4': '',
    #     'A3': '', 'B3': '', 'C3': '', 'D3': 'white-knight', 'E3': '', 'F3': '', 'G3': '', 'H3': '',
    #     'A2': '', 'B2': '', 'C2': '', 'D2': '', 'E2': '', 'F2': '', 'G2': 'black-rook', 'H2': '',
    #     'A1': '', 'B1': '', 'C1': '', 'D1': '', 'E1': '', 'F1': '', 'G1': '', 'H1': ''
    # }
