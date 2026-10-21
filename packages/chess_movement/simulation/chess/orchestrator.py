import signal
import sys
import time
from enum import Enum, auto


class RobotState(Enum):
    IDLE = auto()
    START_TURN = auto()
    HOME = auto()
    WAY_POINT = auto()
    MOVE_HAND_TO_TARGET = auto()
    GRASP = auto()
    LIFT = auto()
    RELEASE = auto()
    FINISH_TURN = auto()
    ERROR = auto()


# Max seconds allowed in each state before going to ERROR. None = no limit.
# logic here: tune these on the real robot
STATE_TIMEOUTS = {
    RobotState.IDLE: None,
    RobotState.START_TURN: 30.0,
    RobotState.HOME: 10.0,
    RobotState.WAY_POINT: 10.0,
    RobotState.MOVE_HAND_TO_TARGET: 10.0,
    RobotState.GRASP: 3.0,
    RobotState.LIFT: 5.0,
    RobotState.RELEASE: 3.0,
    RobotState.FINISH_TURN: 5.0,
    RobotState.ERROR: None,
}

MAX_MAGNET_ON_TIME = 20.0    # s, longest one pick->place should ever take
MAGNET_SETTLE_TIME = 0.2     # s, wait after switching magnet before moving
SAFE_LIFT_HEIGHT_MM = 40.0   # logic here: clear the tallest piece (king) + margin


class Robot:
    def __init__(self):
        self.robot_state = RobotState.IDLE
        self.state_entry_time = time.monotonic()
        self.loop_period = 0.02     # 20 ms tick

        self.plan = []              # remaining (action, square) steps this turn
        self.holding_piece = False
        self.turn_finished = False

        self.magnet_is_on = False
        self.magnet_on_since = None
        self.magnet_switch_time = 0.0

        self.next_motion_id = 0
        self.active_motion_id = None

        self.error_reason = None

    def initialize_robot(self):
        # logic here (connect to ESP32/Teensy, init motors, camera)
        self.magnet_off()           # always start from a known magnet state
        self.enter_idle()

    # ------------------------------------------------------------------ #
    # Helpers: state bookkeeping
    # ------------------------------------------------------------------ #
    def set_state(self, new_state):
        self.robot_state = new_state
        self.state_entry_time = time.monotonic()

    def time_in_state(self):
        return time.monotonic() - self.state_entry_time

    def check_timeout(self):
        limit = STATE_TIMEOUTS.get(self.robot_state)
        return limit is not None and self.time_in_state() > limit

    def next_step(self):
        return self.plan[0] if self.plan else None

    # ------------------------------------------------------------------ #
    # Helpers: magnet
    # ------------------------------------------------------------------ #
    def magnet_on(self):
        # logic here (send MAGNET_ON to ESP32)
        self.magnet_is_on = True
        self.magnet_on_since = time.monotonic()
        self.magnet_switch_time = time.monotonic()

    def magnet_off(self):
        """Must never raise. Always sends OFF, even if we think it's already off."""
        try:
            pass  # logic here (send MAGNET_OFF to ESP32)
        except Exception as e:
            print(f"WARNING: magnet_off failed: {e}")
        self.magnet_is_on = False
        self.magnet_on_since = None
        self.magnet_switch_time = time.monotonic()

    def magnet_settled(self):
        return time.monotonic() - self.magnet_switch_time > MAGNET_SETTLE_TIME

    def send_heartbeat(self):
        # logic here (send heartbeat to ESP32; firmware kills magnet if these stop)
        pass

    # ------------------------------------------------------------------ #
    # Helpers: motion (fix #2 - command IDs so a stale "done" is ignored)
    # ------------------------------------------------------------------ #
    def send_motion(self, target):
        self.next_motion_id += 1
        self.active_motion_id = self.next_motion_id
        # logic here (send motion command tagged with self.active_motion_id)

    def read_completed_motion_id(self):
        # logic here (ID of the last motion the MCU reports as finished)
        return None

    def check_motion_complete(self):
        if self.active_motion_id is None:
            return False
        return self.read_completed_motion_id() == self.active_motion_id

    def stop_all_motion(self):
        """Must never raise."""
        try:
            pass  # logic here (send stop / hold-position to all motors)
        except Exception as e:
            print(f"WARNING: stop_all_motion failed: {e}")
        self.active_motion_id = None

    # ------------------------------------------------------------------ #
    # Helpers: faults
    # ------------------------------------------------------------------ #
    def check_faults(self):
        """Returns a reason string if something is wrong, else None."""
        if (self.magnet_is_on and self.magnet_on_since is not None
                and time.monotonic() - self.magnet_on_since > MAX_MAGNET_ON_TIME):
            return "magnet on too long"
        # logic here (e-stop pressed, motor error flags, lost link to MCU)
        return None

    def emergency_shutdown(self):
        """Safe to call any number of times, from anywhere."""
        print("EMERGENCY SHUTDOWN: magnet off, motion stopped")
        self.magnet_off()
        self.stop_all_motion()

    # ------------------------------------------------------------------ #
    # IDLE
    # ------------------------------------------------------------------ #
    def enter_idle(self):
        print("-> IDLE")
        self.stop_all_motion()
        self.magnet_off()
        self.set_state(RobotState.IDLE)

    def check_our_turn(self):
        # logic here (opponent move detected by camera / button press)
        return False

    def handle_our_turn(self):
        self.enter_start_turn()

    # ------------------------------------------------------------------ #
    # START TURN (request instruction)
    # ------------------------------------------------------------------ #
    def enter_start_turn(self):
        print("-> START_TURN")
        self.set_state(RobotState.START_TURN)
        self.turn_finished = False
        self.holding_piece = False
        self.plan = []
        # logic here (read board, ask Stockfish, start building plan)

    def check_instruction_received(self):
        # logic here (Stockfish move received and converted to self.plan)
        return False

    def handle_instruction_received(self):
        self.enter_home()

    # ------------------------------------------------------------------ #
    # HOME
    # ------------------------------------------------------------------ #
    def enter_home(self):
        print("-> HOME")
        self.set_state(RobotState.HOME)
        self.send_motion("home")    # logic here (home pose)

    def handle_home_reached(self):
        if self.turn_finished:
            self.enter_idle()
        else:
            self.enter_way_point()

    # ------------------------------------------------------------------ #
    # WAY POINT
    # ------------------------------------------------------------------ #
    def enter_way_point(self):
        print("-> WAY_POINT")
        self.set_state(RobotState.WAY_POINT)
        self.send_motion("way_point")   # logic here (safe hover pose above board)

    def handle_way_point_reached(self):
        if self.turn_finished:
            self.enter_home()
        elif not self.plan:
            self.enter_finish_turn()
        else:
            self.enter_move_hand_to_target()

    # ------------------------------------------------------------------ #
    # MOVE HAND TO TARGET
    # ------------------------------------------------------------------ #
    def enter_move_hand_to_target(self):
        step = self.next_step()

        # fix #1: never act on a missing or inconsistent step
        if step is None:
            self.enter_error("move_hand_to_target with empty plan")
            return
        action, square = step
        if action == "pick" and self.holding_piece:
            self.enter_error(f"pick at {square} while already holding a piece")
            return
        if action == "place" and not self.holding_piece:
            self.enter_error(f"place at {square} while not holding a piece")
            return
        if action not in ("pick", "place"):
            self.enter_error(f"unknown action '{action}'")
            return

        print(f"-> MOVE_HAND_TO_TARGET {step}")
        self.set_state(RobotState.MOVE_HAND_TO_TARGET)
        # logic here (travel at SAFE_LIFT_HEIGHT_MM to above square, then descend)
        self.send_motion(("target", square))

    def handle_target_reached(self):
        action, _ = self.next_step()   # validated in enter_move_hand_to_target
        if action == "pick":
            self.enter_grasp()
        else:
            self.enter_release()

    # ------------------------------------------------------------------ #
    # GRASP
    # ------------------------------------------------------------------ #
    def enter_grasp(self):
        print("-> GRASP")
        self.set_state(RobotState.GRASP)
        self.magnet_on()

    def check_grasp_complete(self):
        # logic here (also verify the piece is attached: current/hall/camera)
        return self.magnet_settled()

    def handle_grasp_complete(self):
        if not self.plan:
            self.enter_error("grasp completed with empty plan")
            return
        self.holding_piece = True
        self.plan.pop(0)
        self.enter_lift()

    # ------------------------------------------------------------------ #
    # LIFT (after both grasp and release)
    # ------------------------------------------------------------------ #
    def enter_lift(self):
        print("-> LIFT")
        self.set_state(RobotState.LIFT)
        # logic here (straight vertical move up to SAFE_LIFT_HEIGHT_MM, no sideways motion)
        self.send_motion(("lift", SAFE_LIFT_HEIGHT_MM))

    def handle_lift_complete(self):
        if self.holding_piece:
            # just grasped: carry piece to its destination
            self.enter_move_hand_to_target()
        else:
            # just released
            go_direct = False  # logic here (next pick close enough to skip waypoint?)
            if self.plan and go_direct:
                self.enter_move_hand_to_target()
            else:
                self.enter_way_point()

    # ------------------------------------------------------------------ #
    # RELEASE
    # ------------------------------------------------------------------ #
    def enter_release(self):
        print("-> RELEASE")
        self.set_state(RobotState.RELEASE)
        self.magnet_off()

    def check_release_complete(self):
        # logic here (also verify the piece stayed on the square)
        return self.magnet_settled()

    def handle_release_complete(self):
        if not self.plan:
            self.enter_error("release completed with empty plan")
            return
        self.holding_piece = False
        self.plan.pop(0)
        self.enter_lift()

    # ------------------------------------------------------------------ #
    # FINISH TURN
    # ------------------------------------------------------------------ #
    def enter_finish_turn(self):
        print("-> FINISH_TURN")
        self.set_state(RobotState.FINISH_TURN)
        # logic here (report move to Pi / update board state / hit clock)

    def check_finish_turn_complete(self):
        # logic here
        return True

    def handle_finish_turn_complete(self):
        self.turn_finished = True
        self.enter_way_point()

    # ------------------------------------------------------------------ #
    # ERROR
    # ------------------------------------------------------------------ #
    def enter_error(self, reason):
        print(f"-> ERROR: {reason} (from {self.robot_state.name})")
        self.error_reason = reason
        self.stop_all_motion()
        self.magnet_off()           # drops a held piece; see notes
        self.holding_piece = False
        self.set_state(RobotState.ERROR)

    def check_error_cleared(self):
        # logic here (operator reset: button press / command from Pi UI)
        return False

    def handle_error_cleared(self):
        # Board state is unknown after an error: abort this turn and go home.
        # Operator must fix the board before the next turn.
        print(f"Error cleared: {self.error_reason}")
        self.error_reason = None
        self.plan = []
        self.turn_finished = True   # Home -> Idle
        self.enter_home()

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #
    def robot_loop(self):
        self.send_heartbeat()

        if self.robot_state != RobotState.ERROR:
            fault = self.check_faults()
            if fault:
                self.enter_error(fault)
                return
            if self.check_timeout():
                self.enter_error(
                    f"timeout after {self.time_in_state():.1f}s in {self.robot_state.name}"
                )
                return

        if self.robot_state == RobotState.IDLE:
            if self.check_our_turn():
                self.handle_our_turn()

        elif self.robot_state == RobotState.START_TURN:
            if self.check_instruction_received():
                self.handle_instruction_received()

        elif self.robot_state == RobotState.HOME:
            if self.check_motion_complete():
                self.handle_home_reached()

        elif self.robot_state == RobotState.WAY_POINT:
            if self.check_motion_complete():
                self.handle_way_point_reached()

        elif self.robot_state == RobotState.MOVE_HAND_TO_TARGET:
            if self.check_motion_complete():
                self.handle_target_reached()

        elif self.robot_state == RobotState.GRASP:
            if self.check_grasp_complete():
                self.handle_grasp_complete()

        elif self.robot_state == RobotState.LIFT:
            if self.check_motion_complete():
                self.handle_lift_complete()

        elif self.robot_state == RobotState.RELEASE:
            if self.check_release_complete():
                self.handle_release_complete()

        elif self.robot_state == RobotState.FINISH_TURN:
            if self.check_finish_turn_complete():
                self.handle_finish_turn_complete()

        elif self.robot_state == RobotState.ERROR:
            if self.check_error_cleared():
                self.handle_error_cleared()

        else:
            self.enter_error(f"unknown state {self.robot_state}")


# ---------------------------------------------------------------------- #
# Entry point with magnet failsafe
# ---------------------------------------------------------------------- #
def main():
    robot = Robot()

    # SIGTERM (e.g. systemd stop, `kill`) normally ends the process without
    # running `finally`. Convert it to SystemExit so the finally block runs.
    # Ctrl+C (SIGINT) already raises KeyboardInterrupt, which finally catches.
    def _on_signal(signum, frame):
        raise SystemExit(f"received signal {signum}")

    signal.signal(signal.SIGTERM, _on_signal)

    try:
        robot.initialize_robot()
        while True:
            start = time.monotonic()
            robot.robot_loop()
            time.sleep(max(0.0, robot.loop_period - (time.monotonic() - start)))
    finally:
        robot.emergency_shutdown()


if __name__ == "__main__":
    sys.exit(main())