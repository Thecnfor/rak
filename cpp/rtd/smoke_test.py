#!/usr/bin/env python3
"""rtd --simulate 冒烟测试: 验证 ZMQ JSON 往返 / vel / reset / goto / frame / frame_async / PUB"""
import json, time, threading
import zmq

CMD = "tcp://127.0.0.1:6010"
PUB = "tcp://127.0.0.1:6011"
ctx = zmq.Context()

# ---- REP 客户端 ----
rep = ctx.socket(zmq.REQ)
rep.connect(CMD)
rep.RCVTIMEO = 3000

def cmd(j):
    rep.send_json(j)
    return rep.recv_json()

# 1) state 往返 (state 命令直接返回状态对象, 无 ok 包装)
st = cmd({"cmd": "state"})
assert "x" in st and "mode" in st and "goto_active" in st and "tick_err_ms" in st, st
print("[1] state:", json.dumps(st))

# 2) vel: 前进 0.2 m/s 1.5 秒, 里程计应增长
cmd({"cmd": "vel", "v": [0.2, 0, 0]})
time.sleep(1.5)
st1 = cmd({"cmd": "state"})
assert st1["mode"] == "velocity", st1
assert st1["x"] > 0.05, f"vel 后 x 应增长, got {st1['x']}"
print(f"[2] vel 前进 0.2m/s 1.5s -> x={st1['x']:.3f} y={st1['y']:.3f} dist={st1['dist']:.3f}")

# 3) 看门狗: 不再喂 vel, 应自动零速(里程计不再大幅增长)
x_at_wd = st1["x"]
time.sleep(1.0)
st2 = cmd({"cmd": "state"})
drift = abs(st2["x"] - x_at_wd)
assert drift < 0.02, f"看门狗应停车, x 漂移 {drift}"
print(f"[3] 看门狗 0.5s 自动零速, 1s 后 x 漂移 {drift:.4f} m")

# 4) reset: 指定 x/y/z/distance (state 里角度字段名是 th, reset 输入用 z)
cmd({"cmd": "reset", "x": 1.0, "y": 2.0, "z": 0.5, "distance": 9.0})
st3 = cmd({"cmd": "state"})
assert abs(st3["x"] - 1.0) < 1e-6 and abs(st3["y"] - 2.0) < 1e-6 and abs(st3["th"] - 0.5) < 1e-6 and abs(st3["dist"] - 9.0) < 1e-6, st3
# 部分字段: 只 reset x, 其余保持
cmd({"cmd": "reset", "x": -3.0})
st3b = cmd({"cmd": "state"})
assert abs(st3b["x"] - (-3.0)) < 1e-6 and abs(st3b["y"] - 2.0) < 1e-6 and abs(st3b["th"] - 0.5) < 1e-6, st3b
print(f"[4] reset 全字段/部分字段 -> x={st3b['x']},y={st3b['y']},th={st3b['th']} ok")

# 5) goto: 回到原点并转正(应收敛 ok)
cmd({"cmd": "reset", "x": 0.0, "y": 0.0, "z": 0.0, "distance": 0.0})
r = cmd({"cmd": "goto", "target": [0.3, 0.2, 0.0],
         "max_v": [0.25, 0.25, 0.5], "tol": [0.004, 0.004, 0.02], "timeout": 8})
assert r["ok"] is True, r
print("[5] goto 已异步启动")
t0 = time.time(); done = False
while time.time() - t0 < 10:
    st = cmd({"cmd": "state"})
    if st["mode"] == "idle" and not st["goto_active"]:
        done = True
        print(f"    goto 结束: ok={st['goto_ok']} pos=({st['x']:.4f},{st['y']:.4f},{st['th']:.4f}) 耗时 {time.time()-t0:.2f}s")
        break
    time.sleep(0.05)
assert done, "goto 未在时限内结束"
st = cmd({"cmd": "state"})
assert st["goto_ok"] is True, f"goto 应成功, got {st}"

# 6) frame 同步透传: sim 设备回显 -> 应答匹配
r = cmd({"cmd": "frame", "payload": "0d0200", "timeout_ms": 1000})
print("[6] frame 同步透传:", r)
assert r["ok"] is True and r["payload"] == "0d0200", r

# 7) frame_async -> PUB 回 reply 事件
# 订阅 PUB(subscriber 需先 connect 再等订阅生效)
pub = ctx.socket(zmq.SUB)
pub.connect(PUB)
pub.setsockopt(zmq.SUBSCRIBE, b"")
time.sleep(0.3)  # 等订阅生效
r = cmd({"cmd": "frame_async", "payload": "0d0201", "seq": 42})
assert r["ok"] is True, r
# 等 PUB 上的 reply 事件
got_evt = None
t0 = time.time()
while time.time() - t0 < 3:
    try:
        msg = pub.recv_json(zmq.NOBLOCK)
    except zmq.Again:
        time.sleep(0.02); continue
    if msg.get("evt") == "reply" and msg.get("seq") == 42:
        got_evt = msg
        break
print("[7] frame_async PUB reply 事件:", got_evt)
assert got_evt is not None and got_evt["payload"] == "0d0201"

# 8) frame 超时(应答不会来的指令: 用 motor4 帧, sim 不回显 motor4 -> 超时)
r = cmd({"cmd": "frame", "payload": "010200010203", "timeout_ms": 300})
print("[8] frame 超时:", r)
assert r["ok"] is False, r

# 9) PUB 50Hz 状态流(2 秒内应收到 > 50 条 state)
cnt = 0
t0 = time.time()
while time.time() - t0 < 2.0:
    try:
        msg = pub.recv_json(zmq.NOBLOCK)
    except zmq.Again:
        time.sleep(0.01); continue
    if msg.get("evt") == "state":
        cnt += 1
print(f"[9] PUB state 2s 收到 {cnt} 条 (~50Hz)")
assert cnt >= 80, f"state 频率异常: {cnt}"

# 10) stop -> idle + 零速
cmd({"cmd": "stop"})
st = cmd({"cmd": "state"})
assert st["mode"] == "idle", st
print("[10] stop -> idle ok")

# 11) cancel_goto
cmd({"cmd": "goto", "target": [1.0, 0.0, 0.0]})
cmd({"cmd": "cancel_goto"})
st = cmd({"cmd": "state"})
assert st["mode"] == "idle" and st["goto_active"] is False, st
print("[11] cancel_goto ok")

print("\n=== 冒烟测试全部通过 ===")
