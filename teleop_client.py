# teleop_client.py
import socket
import json
import threading
import time
from pynput import keyboard

UDP_IP = "127.0.0.1"
UDP_PORT = 9876
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 儲存要發送的狀態
state = {
    "vx": 0.0, "vy": 0.0, "wz": 0.0,
    "mode": "recovery",
    "estop": False,
    "clear_estop": False
}

INCREMENT = 0.5
MAX_VEL = 4.0

def send_state():
    try:
        sock.sendto(json.dumps(state).encode('utf-8'), (UDP_IP, UDP_PORT))
    except Exception:
        pass

def on_press(key):
    try:
        k = key.char.lower()
        
        # 前後控制 (VX)
        if k == 'w': 
            state['vx'] = min(state['vx'] + INCREMENT, MAX_VEL)
        elif k == 's': 
            state['vx'] = max(state['vx'] - INCREMENT, -MAX_VEL)
            
        # 左右平移 (VY)
        elif k == 'a': 
            state['vy'] = min(state['vy'] + INCREMENT, MAX_VEL)
        elif k == 'd': 
            state['vy'] = max(state['vy'] - INCREMENT, -MAX_VEL)
            
        # 旋轉控制 (WZ)
        elif k == 'q': 
            state['wz'] = min(state['wz'] + INCREMENT, MAX_VEL)
        elif k == 'e': 
            state['wz'] = max(state['wz'] - INCREMENT, -MAX_VEL)
            
        # 煞車：立刻將目標速度歸零
        elif k == 'z': 
            state['vx'] = state['vy'] = state['wz'] = 0.0
            print("\n[Teleop] Emergency Brake: Velocity reset to 0.")

        # 顯示當前目標速度，方便在終端機觀察
        print(f"\rCurrent Target -> VX: {state['vx']:.1f}, VY: {state['vy']:.1f}, WZ: {state['wz']:.1f}    ", end="")
        send_state()
        
    except AttributeError:
        pass

def on_release(key):
    #nothing
    pass

def heartbeat():
    while True:
        send_state()
        time.sleep(0.1)

def main():
    print("=======================================")
    print("🚀 機器人遠端遙控器 (Remote Teleop)")
    print("=======================================")
    print("鍵盤即時控制 (不需按Enter):")
    print("  [W/S/A/D] 移動   [Q/E] 旋轉   [Z] 煞車")
    print("")
    print("終端機指令模式 (輸入後按Enter):")
    print("  move   : 切換為 LOCOMOTION (行走)")
    print("  stand  : 切換為 pd_stand (站立)")
    print("  re     : 切換為 RECOVERY_STAND (站立)")
    print("  estop  : 緊急停止 (E-Stop)")
    print("  clear  : 解除緊急停止")
    print("  exit   : 關閉遙控器")
    print("=======================================")

    # 在背景啟動鍵盤監聽
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # 啟動心跳包發送
    heart_thread = threading.Thread(target=heartbeat, daemon=True)
    heart_thread.start()

    while True:
        cmd = input("CMD> ").strip().lower()
        if cmd == 'move':
            state['mode'] = 'locomotion'
            print("-> 請求切換至 LOCOMOTION")
        elif cmd == 'stand':
            state['mode'] = 'stand'
            print("-> pd_stand")
        elif cmd == 're':
            state['mode'] = 'recovery'
            print("-> 請求切換至 RECOVERY_STAND (站立)")
        elif cmd == 'estop':
            state['estop'] = True
            print("-> 觸發緊急停止！")
        elif cmd == 'clear':
            state['estop'] = False
            state['clear_estop'] = True
            print("-> 解除緊急停止。")
        elif cmd == 'exit':
            print("遙控器關閉。")
            break
        else:
            print("未知指令。")
        
        send_state()
        state['clear_estop'] = False # 發送完就重置 flag
        
if __name__ == '__main__':
    main()