from simulation.arm_controller import ArmController
from simulation.genesis_setup import GenesisSetup
import os
import time
import numpy as np
from scipy.spatial.transform import Rotation

class SimArmController(ArmController):
    """
    sim_arm_controller.py

    Purpose: Initialize robot configuration and joint controls for URDF model in Genesis Simulation.
    """

# Class Attributes =====================================================================================================

    MAGNET_RANGE = 0.055  # pad->piece-BASE distance cap; tall dowel pieces (king press
    # distance 45.4 mm) need > 0.045. Nearest-piece still disambiguates: at the king
    # press, king=45.4 mm vs queen=51.2 mm; at the knight press, knight=38.3 vs pawn=45.1.

# Constructor ==========================================================================================================

    def __init__(self, scene, robot, urdf_file, arm_name="left", piece_ents=None):
        """
        Constructor: Initialize the ArmController class for one arm of an already spawned robot in simulation scene.

        :param scene: The Genesis Scene from 'genesis_setup'.py object.
        :param robot: The Genesis Robot from 'genesis_setup'.py object.
        :param urdf_file: URDF file (for the IKPY chain).
        :param arm_name: Which arm this controller represents ('right' or 'left').
        :param piece_ents: dict chess square -> piece entity (magnet nearest-piece search).
        """
        super().__init__(urdf_file, arm_name)

        # Initialize parameters
        self.scene = scene
        self.robot = robot
        self.arm_name = arm_name
        self.piece_ents = piece_ents if piece_ents is not None else {}

        self.ee_link = None
        self.arm_dofs_idx_local = []
        self.magnet_dof_idx = None

        # End-effector link (magnet finger) for this arm
        finger_name = "finger_left" if self.arm_name == "left" else "finger_right"
        self.ee_link = self.robot.get_link(finger_name)
        if self.ee_link is None:
            raise ValueError(f"End effector link '{finger_name}' not found in URDF!")

        self.arm_dofs_idx_local = []
        for jn in self.arm["ik_joints"]:
            j = next(j for j in self.robot.joints if j.name == jn)
            self.arm_dofs_idx_local.append(int(j.dofs_idx_local[0]))

        # Rack (finger-extension) dof, looked up by name
        self.magnet_dof_idx = None
        for joint in self.robot.joints:
            if joint.name == self.arm["rack"] and joint.n_dofs > 0:
                self.magnet_dof_idx = joint.dofs_idx_local[0]
                break

        if self.magnet_dof_idx is not None:
            self.robot.set_dofs_kp(np.array([200.0]), np.array([self.magnet_dof_idx]))
            self.robot.set_dofs_kv(np.array([10.0]), np.array([self.magnet_dof_idx]))

        self.attached = None
        self._carry_off = None    # piece base offset in finger frame (m), captured at energize
        self._carry_rot = None    # piece rotation relative to finger, captured at energize
        self._rack_cmd = None     # commanded rack ext while moving (lead-carry snap, see step)
        self.seat_pad_z = None    # pad world Z at energize (release-seat invariant, see toggle_magnet)

# Functions ============================================================================================================

    def read_joints(self):
        """
        Summary: Read the robots current joint positions.

        :return: List of 5 joint angles (rad), in ik_joints order.
        """
        return np.asarray(self.robot.get_dofs_position(self.arm_dofs_idx_local).detach().cpu(), dtype=float)

    def command_joints(self, q):
        """
        Summary: In simulation, apply forces to the joints (actuate the servos) towards target location.

        :param q: Target Joint Angle (rad).
        :return: NA
        """
        self.robot.control_dofs_position(q, self.arm_dofs_idx_local)

    def step(self):
        """
        Summary: Step the simulation forward by one 'tick'. While the magnet holds a
                 piece, snap the held piece itself to the finger's pose (kinematic
                 carry — see toggle_magnet). During a rack move the snap LEADS the
                 pad: the piece is placed where the pad WILL be at the commanded
                 extension, not where it is now. Snapping to the pad's CURRENT pose
                 lags one physics tick behind, so the pad punches the piece's crown
                 every tick and the rack PD (10 N) loses to the fixed piece's
                 infinite mass — the release press stalled ~15 mm early that way.
                 Leading removes the penetration, so the press goes free.

        :return: NA
        """
        self.scene.step()
        if self.attached is not None:
            qf = np.asarray(self.ee_link.get_quat().detach().cpu(), dtype=float)
            pf = np.asarray(self.ee_link.get_pos().detach().cpu(), dtype=float)
            rf = Rotation.from_quat(np.roll(qf, -1))      # wxyz -> scipy xyzw
            lead = 0.0
            if self._rack_cmd is not None:
                # finger moves +1*ext along its local x (URDF axis "1 0 0"; q=0 retracted)
                a = float(self.robot.get_dofs_position([self.magnet_dof_idx]).detach().cpu()[0])
                lead = (self._rack_cmd - a) * 1.0
            self.attached.set_pos(pf + rf.apply(self._carry_off + np.array([lead, 0.0, 0.0])))
            self.attached.set_quat(np.roll((rf * self._carry_rot).as_quat(), 1))  # xyzw -> wxyz

    def move_rack(self, v, v_max=None):
        """
        Summary: Drive the rack to absolute extension v with a trapezoidal speed
                 profile (same movement_profile the arm uses — it is unit-agnostic,
                 meters here instead of radians), then HOLD it there until it settles
                 (like the arm's settle — a fixed tick count sampled the PD
                 mid-oscillation and made presses non-repeatable). Presses against a
                 piece can never reach v, so this caps the wait instead of asserting it.

        :param v: target rack extension (m; 0 = pad highest/retracted, RACK_RETRACTED = pad lowest).
        :param v_max: cruise speed m/s (None = RACK_SPEED).
        :return: NA
        """
        v_max = self.RACK_SPEED if v_max is None else v_max
        q0 = float(self.robot.get_dofs_position([self.magnet_dof_idx]).detach().cpu()[0])
        delta = v - q0
        if abs(delta) < 1e-6:
            return
        for frac in self.movement_profile(abs(delta), v_max, self.RACK_ACCEL, self.DT):
            self._rack_cmd = q0 + frac * delta
            self.robot.control_dofs_position([self._rack_cmd], [self.magnet_dof_idx])
            self.step()    # self.step (not scene.step): keeps the carry-snap live mid-press
        # Capped settle: hold the final setpoint; a press against a piece never reaches it.
        for _ in range(300):
            self._rack_cmd = v
            self.robot.control_dofs_position([v], [self.magnet_dof_idx])
            self.step()    # self.step (not scene.step): keeps the carry-snap live mid-press
            if abs(float(self.robot.get_dofs_position([self.magnet_dof_idx]).detach().cpu()[0])
                   - v) < 0.0005:
                break
        self._rack_cmd = None

    @staticmethod
    def _quat_to_rot(q):
        """(Helper) (w, x, y, z) quaternion -> 3x3 rotation matrix."""
        w, x, y, z = (float(v) for v in q)
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])

    def pad_world_z(self):
        """
        Summary: World Z of the pad face (finger pose + TCP_LOCAL offset) — the same
                 point toggle_magnet measures the seat invariant with.

        :return: pad face world Z (m).
        """
        qf = np.asarray(self.ee_link.get_quat().detach().cpu(), dtype=float)
        pf = np.asarray(self.ee_link.get_pos().detach().cpu(), dtype=float)
        return float((pf + self._quat_to_rot(qf) @ np.asarray(self.TCP_LOCAL, dtype=float))[2])

    def read_rack(self):
        """
        Summary: Read the rack's current extension.

        :return: current rack extension (m; 0 = pad highest/retracted, RACK_RETRACTED = lowest).
        """
        return float(self.robot.get_dofs_position([self.magnet_dof_idx]).detach().cpu()[0])

    def toggle_magnet(self) -> bool:
        """
        Summary: Energize/de-energize the electromagnet (kinematic carry: the grabbed
                 piece itself is ghosted (no contacts) and snapped to the finger every
                 step by step(); on release it is un-ghosted right where the carry left
                 it). Genesis welds proved too soft to carry through arm swings.
                 Call when the rack is already pressed (pad at the piece crown).

        :return: True if a piece attached (energize) or detached (de-energize);
                 False if energize found nothing in range.
        """
        if self.attached is None:                                   # -> energize
            pf = np.asarray(self.ee_link.get_pos().detach().cpu(), dtype=float)
            qf = np.asarray(self.ee_link.get_quat().detach().cpu(), dtype=float)
            pad = pf + self._quat_to_rot(qf) @ np.asarray(self.TCP_LOCAL, dtype=float)
            best, best_d, best_pose = None, self.MAGNET_RANGE, None
            for ent in self.piece_ents.values():
                pp = np.asarray(ent.get_pos().detach().cpu(), dtype=float)
                d = float(np.linalg.norm(pp - pad))
                if d < best_d:
                    best, best_d = ent, d
                    best_pose = (pp, np.asarray(ent.get_quat().detach().cpu(), dtype=float))
            if best is None:
                print("magnet: nothing in range")
                return False
            # Release-seat invariant: the piece's base sat on the surface at this
            # instant, so the pad's Z HERE is the exact Z release must return to.
            # The surface constant cancels — landing is exact regardless of hull
            # early-stall, arm PD sag, or which piece is held.
            self.seat_pad_z = float(pad[2])
            # Capture the piece's pose RELATIVE TO THE FINGER; step() replays it every tick.
            qf = np.asarray(self.ee_link.get_quat().detach().cpu(), dtype=float)
            pf = np.asarray(self.ee_link.get_pos().detach().cpu(), dtype=float)
            rf = Rotation.from_quat(np.roll(qf, -1))                # wxyz -> scipy xyzw
            rp = Rotation.from_quat(np.roll(best_pose[1], -1))
            self._carry_off = rf.inv().apply(best_pose[0] - pf)
            self._carry_rot = rf.inv() * rp
            for geom in best.geoms:               # ghost while carried: no contact forces
                geom._contype = 0
                geom._conaffinity = 0
            self.attached = best
            return True
        else:                                         # -> de-energize
            for geom in self.attached.geoms:      # un-ghost: already at its landing pose
                geom._contype = 1
                geom._conaffinity = 1
            self.attached = None
        return True

# Main =================================================================================================================

def main():
    # Create Scene with 'genesis_setup' --------------------------------------------------------------------------------

    # Set URDF
    _SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    urdf_file = os.path.join(_SCRIPT_DIR, "master_assembly_v6", "urdf", "master_assembly_v6.urdf")

    # Run Simulation
    sim = GenesisSetup(urdf_file)
    sim.setup()

    # Use 'arm_controller'.py class ------------------------------------------------------------------------------------

    # Create Objects
    l_ctrl = SimArmController(sim.scene, sim.robot, urdf_file, "left", sim.piece_ent)
    r_ctrl = SimArmController(sim.scene, sim.robot, urdf_file, "right", sim.piece_ent)

    # Hold: step for 60 s so the viewport stays open for inspection (the viewer only
    # lives while the script runs — without this loop Genesis exits and closes it).
    start = time.time()
    while time.time() - start < 60.0:
        sim.scene.step()

if __name__ == "__main__":
    main()