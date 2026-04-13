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
        self.is_started = False    # 1st stand up
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
                start = input("Start and stand up? (y/n): ").lower()
                self.is_started = True if (start == 'y') else None
            else:
                cmd = input("\n[move] to start control / [exit] to quit: ").lower()
                if cmd == "exit":
                    self.is_exit = True
                    break
                elif cmd == "move":
                    if not self.is_moving:
                        print("Start moving. Use WASD+Q/E to control, 'z' to stop, 'c' to exit.")
                        self.is_moving = True
                    else:
                        print("Already in move mode.")