"""VOFA+ 连通性测试 — 每目标单独一帧，XY散点图可见全部点"""
import socket
import struct
import time
import math

HOST = "127.0.0.1"
PORT = 1347
TAIL = b'\x00\x00\x80\x7F'

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print(f"VOFA+ test: JustFloat -> {HOST}:{PORT}")
print("VOFA+ 中：JustFloat → UDP → 端口 1347 → 连接 → XY图 X=I0 Y=I1")
print("按 Ctrl+C 停止")
print()

t = 0
try:
    while True:
        # 目标1：绕圈船 A
        x = 0.3 * math.cos(t * 0.5)
        y = 2.0 + 0.3 * math.sin(t * 0.5) + t * 0.02
        sock.sendto(struct.pack('<ffff', x, y, 0.3, 50.0) + TAIL, (HOST, PORT))

        # 目标2：绕圈船 B
        x = 0.7 + 0.3 * math.cos(t * 0.5 + math.pi)
        y = 2.0 + 0.3 * math.sin(t * 0.5 + math.pi) + t * 0.02
        sock.sendto(struct.pack('<ffff', x, y, 0.3, 50.0) + TAIL, (HOST, PORT))

        # 目标3-5：杂波点（在附近随机晃动）
        for k in range(3):
            x = -0.5 + 0.8 * math.sin(t * 1.7 + k)
            y = 1.0 + 2.0 * (0.5 + 0.5 * math.cos(t * 1.3 + k * 2))
            sock.sendto(struct.pack('<ffff', x, y, 0.0, 20.0) + TAIL, (HOST, PORT))

        time.sleep(0.08)  # ~12.5Hz，每帧发5个包
        t += 0.08

except KeyboardInterrupt:
    print("\nStopped.")
    sock.close()
