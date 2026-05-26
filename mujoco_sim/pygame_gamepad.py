import itertools
import threading
import pygame
from MPC_Controller.utils import GaitType, FSM_StateName

ALLOWED_MODES = [FSM_StateName.RECOVERY_STAND, FSM_StateName.LOCOMOTION]
ALLOWED_GAITS = [x for x in GaitType]

INCREMENT = 0.1
MAX_VEL   = 3.0


class PyGamepad:
    """Pygame keyboard gamepad — drop-in replacement for UDPGamepad.

    Key bindings (mirroring RL_Environment/gamepad_reader.py):
      W/S        left-stick Y   → vx forward/back
      A/D        right-stick X  → vy strafe
      Q/E        left-stick X   → wz rotate
      Z          —              → brake (zero all velocity)
      Enter      RB release     → toggle stand ↔ move
      G          LB release     → cycle gait
      Esc / X    LB+RB          → e-stop
      C          LJ click       → clear e-stop
    """

    def __init__(self, vel_scale_x: float = 1.0,
                 vel_scale_y: float = 1.0,
                 vel_scale_rot: float = 1.5):
        self.vx  = 0.0
        self.vy  = 0.0
        self.wz  = 0.0
        self._vel_scale_x   = vel_scale_x
        self._vel_scale_y   = vel_scale_y
        self._vel_scale_rot = vel_scale_rot

        self._estop_flagged = False
        self.is_standing    = False
        self.is_moving      = False

        self._gait_generator = itertools.cycle(ALLOWED_GAITS)
        self._gait           = next(self._gait_generator)
        self._mode_generator = itertools.cycle(ALLOWED_MODES)
        self._mode           = next(self._mode_generator)  # starts at RECOVERY_STAND

        self.running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        print("[PyGamepad] Pygame keyboard gamepad started. Click the teleop window to control.")

    # ── public API (same as UDPGamepad / Gamepad) ────────────────────

    def get_command(self):
        return (self.vx, self.vy, 0.0), self.wz, self._estop_flagged

    def get_gait(self):
        return self._gait

    def get_mode(self):
        return self._mode

    def fake_event(self, ev_type, code, value):
        """Called by main loop to programmatically inject events (e.g. clear e-stop)."""
        if ev_type == 'Key' and code == 'BTN_TR' and value == 0:
            self._estop_flagged = False

    def stop(self):
        self.running = False
        self._thread.join(timeout=1.0)
        print("[PyGamepad] Stopped.")

    # ── internal ─────────────────────────────────────────────────────

    def _enter_stand(self):
        self._mode       = FSM_StateName.RECOVERY_STAND
        self.is_standing = True
        self.is_moving   = False
        self.vx = self.vy = self.wz = 0.0

    def _enter_move(self):
        if self._estop_flagged:
            return
        self._mode       = FSM_StateName.LOCOMOTION
        self.is_standing = False
        self.is_moving   = True

    def _estop(self):
        self._estop_flagged = True
        self.vx = self.vy = self.wz = 0.0
        self.is_moving   = False
        self.is_standing = True
        self._mode = FSM_StateName.RECOVERY_STAND
        print("[PyGamepad] E-Stop! Press C to clear.")

    def _clear_estop(self):
        self._estop_flagged  = False
        self._mode_generator = itertools.cycle(ALLOWED_MODES)
        self._mode           = next(self._mode_generator)
        print("[PyGamepad] E-Stop cleared.")

    def _handle_key(self, key):
        if self._estop_flagged and key != pygame.K_c:
            return  # block all movement keys while estopped

        if   key == pygame.K_w:      self.vx = min(self.vx + INCREMENT, MAX_VEL)
        elif key == pygame.K_s:      self.vx = max(self.vx - INCREMENT, -MAX_VEL)
        elif key == pygame.K_a:      self.vy = min(self.vy + INCREMENT, MAX_VEL)
        elif key == pygame.K_d:      self.vy = max(self.vy - INCREMENT, -MAX_VEL)
        elif key == pygame.K_q:      self.wz = min(self.wz + INCREMENT, MAX_VEL)
        elif key == pygame.K_e:      self.wz = max(self.wz - INCREMENT, -MAX_VEL)
        elif key == pygame.K_z:      self.vx = self.vy = self.wz = 0.0
        elif key == pygame.K_t:      self._enter_stand()
        elif key == pygame.K_m:      self._enter_move()
        elif key == pygame.K_g:      self._gait = next(self._gait_generator)
        elif key in (pygame.K_ESCAPE, pygame.K_x): self._estop()
        elif key == pygame.K_c:      self._clear_estop()

    def _run(self):
        pygame.init()
        screen = pygame.display.set_mode((420, 190))
        pygame.display.set_caption("Robot Teleop")
        font_b = pygame.font.SysFont(None, 24)
        font_s = pygame.font.SysFont(None, 20)
        clock  = pygame.time.Clock()

        GREY  = (200, 200, 200)
        RED   = (255,  80,  80)
        GREEN = ( 80, 220, 100)
        DIM   = (120, 120, 120)
        BG    = ( 25,  25,  25)

        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    self._handle_key(event.key)

            # ── HUD ──────────────────────────────────────────────────
            screen.fill(BG)

            if self.is_moving:
                mode_str, mode_col = "MOVE  (MPC)", GREEN
            elif self.is_standing:
                mode_str, mode_col = "STAND (pd)",  GREY
            else:
                mode_str, mode_col = "IDLE",        DIM
            estop_col = RED if self._estop_flagged else DIM
            gait_name = self._gait.name if hasattr(self._gait, "name") else str(self._gait)

            lines = [
                (f"Mode : {mode_str}",                            mode_col),
                (f"Gait : {gait_name}",                           GREY),
                (f"Vx {self.vx:+.1f}  Vy {self.vy:+.1f}  Wz {self.wz:+.1f}", GREY),
                (f"EStop: {'YES — press C to clear' if self._estop_flagged else 'off'}", estop_col),
                ("", DIM),
                ("W/S forward  A/D strafe  Q/E rotate  Z brake",  DIM),
                ("T: stand  M: move  G: gait  Esc/X: estop  C: clear", DIM),
            ]
            for i, (text, col) in enumerate(lines):
                f = font_b if i < 4 else font_s
                screen.blit(f.render(text, True, col), (10, 8 + i * 26))

            pygame.display.flip()
            clock.tick(30)

        pygame.quit()
