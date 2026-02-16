"""Simple USB serial to ESP32 for lead screw positions. Sends row 1-8 for moves, 9 for clock, 10 for opponent."""

import serial
import time

PORT = "/dev/ttyUSB0"  # change this
BAUD = 115200

ser = None


def open_serial():
    """Open serial connection. Call once at startup."""
    global ser
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)  # ESP32 often resets when serial opens


def close_serial():
    """Close serial connection."""
    global ser
    if ser:
        ser.close()
        ser = None


def write_position(pos: int):
    """Send position P1-P10 to ESP32."""
    if ser and ser.is_open:
        ser.write(f"P{pos}\n".encode())


def send_move(move: str):
    """Send the row for a robot move (1-8) to ESP32."""
    row = int(move[-1])
    write_position(row)


def send_clock_position():
    """Position 9: robot hits clock for opponent's turn."""
    write_position(9)


def send_opponent_position():
    """Position 10: opponent's turn."""
    write_position(10)


if __name__ == "__main__":
    open_serial()
    write_position(3)  # test: e3
    time.sleep(0.5)
    close_serial()
