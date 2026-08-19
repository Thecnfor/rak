// zmq.rs —— ZMQ 接口层: REP 命令线程(std) + PUB 状态/事件任务(tokio async)
//
// 与 cpp/rtd 的 ZmqServer 完全同协议:
//   REP tcp://127.0.0.1:6010  (JSON)
//     {"cmd":"vel","v":[x,y,z]}                    -> {"ok":true}
//     {"cmd":"goto","target":[x,y,th],"max_v":..,"tol":..,"timeout":..} -> {"ok":true}
//     {"cmd":"cancel_goto"} / {"cmd":"stop"}
//     {"cmd":"reset","x":..,"y":..,"z":..,"distance":..}
//     {"cmd":"state"}                              -> {"x","y","th","dist","mode",
//                                                          "goto_active","goto_ok","tick_err_ms"}
//     {"cmd":"frame","payload":"hex","timeout_ms":200} -> {"ok":true,"payload":"hex"} / {"ok":false}
//     {"cmd":"frame_async","payload":"hex","seq":N}    -> {"ok":true}, 应答/超时走 PUB
//     {"cmd":"sub","dev":d,"mode":m,"port":p}          -> {"ok":true}, 订阅帧走 PUB evt:frame
//
//   PUB tcp://127.0.0.1:6011  (50Hz 状态 + 事件)
//     {"evt":"state", ...同 state}
//     {"evt":"reply","seq":N,"payload":"hex"} / {"evt":"timeout","seq":N}
//     {"evt":"frame","dev":..,"mode":..,"port":..,"payload":"hex"}
//
// 设计说明: 100Hz 控制环与 ZMQ REP 都必须在专用线程(阻塞调用/实时节拍),
// 所以 REP 用 std 线程; PUB 推送是纯 IO 节奏(50Hz 节拍 + 事件即时), 用 tokio 任务。

use crate::core::{Core, handle_command};
use crate::log_e;
use crate::log_i;
use crate::util::mono_now;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::mpsc::UnboundedReceiver;

/// REP 线程: 独占 REP socket, 循环收命令/回 JSON(与 cpp/rtd 的 rep_loop 一致)。
pub fn rep_loop(core: Arc<Mutex<Core>>, cmd_url: &str) {
    let ctx = zmq::Context::new();
    let sock = match ctx.socket(zmq::REP) {
        Ok(s) => s,
        Err(e) => {
            log_e!("创建 REP socket 失败: {}", e);
            return;
        }
    };
    // 接收超时, 便于停机时检查 stop 退出
    if let Err(e) = sock.set_rcvtimeo(100) {
        log_e!("REP 设置 rcvtimeo 失败: {}", e);
        return;
    }
    if let Err(e) = sock.bind(cmd_url) {
        log_e!("REP bind {} 失败: {}", cmd_url, e);
        return;
    }
    log_i!("REP 监听 {}", cmd_url);

    loop {
        let stopped = core.lock().unwrap().stop;
        if stopped {
            break;
        }
        let req = match sock.recv_msg(0) {
            Ok(m) => m,
            Err(_) => continue, // 100ms 接收超时, 回到循环检查 stop
        };
        let in_str = match req.as_str() {
            Some(s) => s.to_string(),
            None => String::new(),
        };
        let rep = handle_command(&core, &in_str);
        if let Err(e) = sock.send(&rep, 0) {
            log_e!("REP 发送失败: {}", e);
        }
    }
    log_i!("REP 线程退出");
}

/// PUB 任务(tokio): 事件(应答/超时/订阅帧)即时推送, 状态 50Hz 心跳。
/// pub_url 传入 String(tokio task 需要 'static, 由调用方 move 进来)。
pub async fn pub_loop(
    core: Arc<Mutex<Core>>,
    pub_url: String,
    mut rx: UnboundedReceiver<String>,
) {
    let ctx = zmq::Context::new();
    let sock = match ctx.socket(zmq::PUB) {
        Ok(s) => s,
        Err(e) => {
            log_e!("创建 PUB socket 失败: {}", e);
            return;
        }
    };
    if let Err(e) = sock.bind(&pub_url) {
        log_e!("PUB bind {} 失败: {}", pub_url, e);
        return;
    }
    log_i!("PUB 监听 {} (50Hz 状态)", pub_url);

    let mut interval = tokio::time::interval(Duration::from_millis(20));
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

    loop {
        let exit = tokio::select! {
            // 事件到达即时推送(低延迟 reply/timeout/frame)
            evt = rx.recv() => match evt {
                Some(s) => {
                    let _ = sock.send(&s, 0);
                    // 顺带清空积压, 避免事件堆积
                    while let Ok(e) = rx.try_recv() {
                        let _ = sock.send(&e, 0);
                    }
                    false
                }
                None => true, // 发送端全部关闭 -> 退出
            },
            // 50Hz 状态心跳
            _ = interval.tick() => {
                if core.lock().unwrap().stop {
                    true // 停机 -> 退出
                } else {
                    while let Ok(e) = rx.try_recv() {
                        let _ = sock.send(&e, 0);
                    }
                    let st = core.lock().unwrap().state_json(true);
                    let out = st.to_string();
                    let _ = sock.send(&out, 0);
                    false
                }
            }
        };
        if exit {
            break;
        }
    }
    log_i!("PUB 任务退出");
}

/// 打印启动横幅(供 main 使用)。
pub fn startup_banner(
    tick_hz: u64,
    port: &str,
    cmd_port: u16,
    pub_port: u16,
    simulate: bool,
    started_at: f64,
) {
    log_i!(
        "rtd 已启动: tick={}Hz, port={}, cmd={}, pub={}, simulate={} (启动耗时 {:.1}s)",
        tick_hz,
        port,
        cmd_port,
        pub_port,
        if simulate { "on" } else { "off" },
        mono_now() - started_at
    );
    if simulate {
        log_i!("警告: simulate 模式未连接真实串口, 仅供协议联调");
    } else {
        log_i!("警告: 当前将连接真实串口。上车部署请用 systemd 管理并确认 Python 进程已停止");
    }
}
