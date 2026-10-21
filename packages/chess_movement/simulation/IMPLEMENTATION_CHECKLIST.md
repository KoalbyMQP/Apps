# Chess Sim — Implementation Checklist

Goal: interactive Genesis chess game where the magnet-hand URDF physically picks,
carries, and places pieces, viewer visible, driven by ChessOrchestrator.
Reference package for techniques: mmh_chess (professor's Claude-Pro benchmark repo).

Verified facts (probed against the project venv, Genesis 0.4.6):
- rigid solver HAS add_weld_constraint / delete_weld_constraint  -> magnet model
- RigidOptions accepts max_dynamic_constraints
- Entity HAS inverse_kinematics, forward_kinematics, control_dofs_position,
  get_link, get_joint
- python-chess 1.11.2 installed in venv

Coordinate frames: YOURS = floor origin, robot base z=0.735, BOARD_TOP_Z=0.62,
BOARD_Y0=0.135, board_length=0.2032. mmh_chess uses chest origin / board at
z=-0.215 — copy their FORMULAS, never their constants.

---

## Phase A — World model (simulation/genesis_setup.py)

- [ ] A1. Construct with physics_pieces=True everywhere the sim is built.
- [ ] A2. Calibrate piece density: Genesis takes rho, not mass.
      rho = target_mass / convex_hull_volume (needs trimesh, or approximate
      hull volume from PIECE_DIMS cylinder). Add a mass table (mmh_chess:
      pawn 12 g, rook/knight 16 g, bishop 15 g, queen 18 g, king 19 g —
      includes steel washer).
- [ ] A3. Add RigidOptions(max_dynamic_constraints=4) to the gs.Scene so the
      runtime magnet weld has constraint slots.
- [ ] A4. Graveyard strips (user design): widen the board slab to 12x8
      (width = 12*SQUARE + margin, depth stays 8*SQUARE). The outer two
      columns on each side are NOT checker tiles — separate visual strips
      (plain boxes, no pattern) giving each team 2 cols x 8 rows = 16 slots.
      Collision comes free from the fixed slab; strips sit at BOARD_TOP_Z.
      Slot centers: x = +/- (board_length/2 + 0.5*SQUARE) [inner col] and
      +/- (board_length/2 + 1.5*SQUARE) [outer col]; y = BOARD_Y0 + (r+0.5)*SQUARE.
      Slot i (0..15): column = i//8, row = i%8.
      Expose a table: graveyard_slot(team, i) -> world (x, y).
      NOTE: far strip (x < 0) may be out of reach for a single right arm
      (mmh_chess reach map ends ~file h). Assign team->side after checking
      reach; single-arm interim: capture into the arm's own-side strip.
- [ ] A5. Delete the 10 s settle loop and 60 s idle loop in setup().
      Replace with ~0.3 s settle (pieces come to rest), then RETURN control.
      setup() must not consume the stepping loop.
- [ ] A6. Expose what the game loop needs: scene, robot, piece_ent {square ->
      entity}, BOARD_TOP_Z, SQUARE, piece heights (already tracked in piece_h).

Acceptance: run genesis_setup main; viewer opens, robot + board + pieces spawn,
pieces rest on board, program exits cleanly after settle.

## Phase B — MovementSystem rewrite (simulation/chess/movement_system.py)

- [ ] B1. Replace set_qpos teleporting with control_dofs_position + scene
      stepping. target() interpolates setpoints from current to goal
      (min-jerk s^3(10-15s+6s^2) like mmh_chess, or linear to start),
      calling scene.step() each tick. MovementSystem needs a scene reference.
- [ ] B2. PD gains + force clamps per joint group (start values from
      mmh_chess: kp 60/kv 1.0 arm joints, kp 20/kv 0.3 wrist/hand/rack,
      rack kp 2000/kv 20, force range = stall torques 7.6/2.35 N·m,
      rack 20 N).
- [ ] B3. Rack-based grasp (hardware truth: the 32 mm prismatic rack does the
      reaching, not the arm):
        hover_z = BOARD_TOP_Z + piece_h + FINGER_TRAVEL(0.032) - press(0.0015)
      grasp sequence: arm to hover over piece -> fingerextend_right extends
      rack down onto piece crown -> magnet ON -> rack retracts (lifts piece).
      release sequence: rack extends to set piece down ~0.5 mm above board ->
      magnet OFF -> rack retracts.
- [ ] B4. Magnet = weld constraint, not lerp:
        attach: scene.sim.rigid_solver.add_weld_constraint(
                    finger_link.idx, piece_entity.base_link.idx)
        detach: delete_weld_constraint(same args)
      Engage only when magnet-on AND pad-to-piece-top gap < 0.004 m.
      Finger link: get_link("finger_right"); pad face offset in finger frame
      ~[0.083, 0, -0.0075] (verify against YOUR urdf — the local pad center).
- [ ] B5. Entity bridge: grasp/release must know WHICH piece. Add
      MovementSystem.attach_entity(piece_entity) / detach_entity(), and have
      the orchestrator pass the entity (from GenesisSetup.piece_ent[square]).
      Keep movement step-signature: chess code stays in board coords.
- [ ] B6. home()/waypoint() also become stepped motions (same interpolation).
- [ ] B7. IK: keep robot.inverse_kinematics(link=...) but verify the approach
      stays vertical (rack axis pointing down). mmh_chess enforced this with a
      6-row DLS; Genesis IK may drift orientation. If pieces get hit on
      approach, add a wrist-only correction or re-seed IK from previous qpos.

Acceptance: with physics_pieces=True and viewer on, call movement.grasp on the
e2 pawn entity -> rack visibly extends, pawn sticks (weld), rack retracts
lifting pawn; release sets it down without dragging.

## Phase C — ChessOrchestrator wiring (simulation/chess/chess_operations.py)

Already fixed (done this session): _physically_move split, graveyard coord
bug, remove() board sync, promote() bookkeeping, castle() bookkeeping,
en_passant verified. Remaining:

- [ ] C1. Thread the piece entity through the physical path: move() looks up
      entity for start_location, passes it to grasp/release. Same for
      remove() (carries captured piece to graveyard tray) and promote()
      (piece FROM graveyard tray — needs an entity for graveyard slots too).
- [ ] C2. Z constants injected from genesis_setup (lift/carry height, hover
      formula), no hardcoded 0.7.
- [ ] C3. Graveyard becomes TWO queues (one per team, 16 slots each), fed by
      genesis_setup.graveyard_slot(team, i). Entries: (piece_type, entity,
      slot). remove() routes a captured piece to the owning team's queue
      (piece.color decides team — the dead piece goes where its army's dead
      go, per your design). promote() searches the PROMOTING side's own
      queue and physically fetches that entity back onto the board.
      Drop the old GRAVEYARD_X0/Y/PITCH constants — slot positions now come
      from the setup table.
- [ ] C4. Carry height: hover_z + 0.025 transport lift (clears king height).

Acceptance: mock-movement unit tests still pass (test_pickup_move.py), plus a
real-sim take() (exd5) where black pawn lands in tray and white pawn lands on d5.

## Phase D — Game loop (simulation/chess/main.py)

- [ ] D1. Composition root:
        sim = GenesisSetup(urdf, physics_pieces=True)
        sim.setup()
        movement = MovementSystem(sim.robot, sim.scene)
        game = ChessOrchestrator(movement, sim.board_length, sim.BOARD_Y0,
                                 pieces=sim.piece_ent, ...)
- [ ] D2. Match loop: get next move (stdin SAN or scripted list) ->
      game.move/take/... -> scene keeps stepping during animation (stepping
      lives inside movement calls; loop keeps stepping ~0.5 s after each move
      so physics settles) -> next move. Viewer stays open the whole match.
- [ ] D3. (Optional) Stockfish opponent: chess.engine.SimpleEngine.popen_uci
      with the stockfish binary; filter candidate moves by reachability.
- [ ] D4. (Optional, later) Reachability filter: ranks 7-8 unreachable
      without torso lean (mmh_chess: 44/64 squares). Until then, restrict
      scripted games to ranks 1-5 or accept IK failures.

Acceptance: full scripted 4-ply game in the viewer; every piece physically
relocated; board state and logic board agree at the end.

## Phase E — Validation & tuning

- [ ] E1. Adopt mmh_chess success criteria per move: placed within 6 mm of
      square center; other pieces displaced < 0.1 mm.
- [ ] E2. Tuning order if it misbehaves: (1) PD gains/force range, (2) hover_z
      press depth, (3) capture distance 4 mm, (4) place hover 0.5 mm (pressing
      into board drags pieces sideways on release), (5) rack speed.
- [ ] E3. Known risks: Genesis convex hulls fatter than meshes (piece
      clearance); IK orientation drift (B7); weld + high-speed carry = swing
      (keep carry slow); graveyard tray collision must exist or pieces fall
      through the world.

## Environment notes

- Use the project venv: "My Code/Complete Chess/Chess Movement/.venv"
  (genesis 0.4.6, mujoco 3.13, chess 1.11.2). Do NOT pip-install the guide's
  pinned versions (genesis-world 1.4.4 does not exist on PyPI; guide's adapter
  code targets an API your 0.4.6 lacks in places, e.g. vis.DirectionalLight).
- trimesh needed only if using the hull-volume density trick.