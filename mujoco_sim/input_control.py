# input_control.py
import threading
from pynput import keyboard

class InputHandler:
    def __init__(self):
        # --- 速度指令 ---
        self.vx = 0.0
        self.vy = 0.0
        self.angv = 0.0

        # --- 狀態 ---
        self.is_started = False 
        self.is_standing = False    # 1st stand up
        self.is_moving = False     
        self.is_exit = False   

    def start(self):
        """activate threads"""
        # 1. keyboard thread
        self.listener = keyboard.Listener(on_press=self._on_press)
        self.listener.start()

        # 2. cmd line thread
        self.cmd_thread = threading.Thread(target=self._cmd_loop, daemon=True)
        self.cmd_thread.start()

    def _on_press(self, key):
        """ WASD """
        if not self.is_moving:
            return 

        try:
            k = key.char.lower()
            if k == 'w': self.vx = 0.5
            elif k == 's': self.vx = -0.5
            elif k == 'a': self.vy = 0.5
            elif k == 'd': self.vy = -0.5
            elif k == 'q': self.angv = 0.5
            elif k == 'e': self.angv = -0.5
            elif k == 'z': # 煞車
                self.vx = self.vy = self.angv = 0.0
            elif k == 'c': # 退出移動模式
                print("\n[Keyboard] Exiting move mode.")
                self.vx = self.vy = self.angv = 0.0
                self.is_moving = False
        except AttributeError:
            pass

    def _cmd_loop(self):
        """處理終端機文字指令"""
        while not self.is_exit:
            if not self.is_started:
                start = input("Start simulation and stand up? (y/n): ").strip().lower()
                if start == 'y':
                    self.is_started, self.is_standing = True, True
            else:
                if self.is_moving:
                    continue
                cmd = input("\nCommands: [move] to start control / [exit] to quit: ").strip().lower()
                if cmd == "exit":
                    self.is_exit = True
                    break
                elif cmd == "move" and (not self.is_moving):
                    print("Move mode ENABLED. Use WASD to control, 'z' to stop, 'c' to exit.")
                    self.is_moving = True
                    self.is_standing = False