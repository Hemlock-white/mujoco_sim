# RL_Environment/udp_reader.py
import socket
import threading
import json
from MPC_Controller.utils import GaitType, FSM_StateName

class UDPGamepad:
    """
    這是一個 Drop-in replacement，用來完美取代原本的 Gamepad 類別。
    它會在背景開啟一個 UDP Server 接收來自另一個終端機的指令。
    """
    def __init__(self, port=9876):
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self._mode = FSM_StateName.RECOVERY_STAND
        self._gait = GaitType.TROT
        self._estop_flagged = False
        self.is_standing = False
        self.is_moving = False

        # 設定 UDP Server
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', port))
        self.sock.settimeout(0.05) # 0.05 秒 non-blocking

        self.running = True
        self.thread = threading.Thread(target=self._listen, daemon=True)
        self.thread.start()
        print(f"[UDP_Reader] Listening for remote commands on port {port}...")

    def _listen(self):
        """在背景不斷接收封包"""
        while self.running:
            try:
                data, _ = self.sock.recvfrom(1024)
                msg = json.loads(data.decode('utf-8'))
                
                # 更新速度
                self.vx = msg.get('vx', self.vx)
                self.vy = msg.get('vy', self.vy)
                self.wz = msg.get('wz', self.wz)
                
                # 更新狀態機模式
                mode_str = msg.get('mode', '')
                if mode_str == 'locomotion':
                    self._mode = FSM_StateName.LOCOMOTION
                    self.is_standing = False
                    self.is_moving = True
                elif mode_str == "stand":
                    self.is_standing = True
                    self.is_moving = False
                elif mode_str == 'recovery':
                    self._mode = FSM_StateName.RECOVERY_STAND
                
                # 緊急停止狀態
                if msg.get('estop', False):
                    self._estop_flagged = True
                    self.vx = self.vy = self.wz = 0.0
                    self._mode = FSM_StateName.RECOVERY_STAND
                if msg.get('clear_estop', False):
                    self._estop_flagged = False

            except socket.timeout:
                pass
            except Exception:
                pass

    def get_command(self):
        return (self.vx, self.vy, 0.0), self.wz, self._estop_flagged

    def get_gait(self):
        return self._gait

    def get_mode(self):
        return self._mode

    def fake_event(self, ev_type, code, value):
        # 原本用來清除 gamepad 鎖死的邏輯
        if ev_type == 'Key' and code == 'BTN_TR' and value == 0:
            self._estop_flagged = False