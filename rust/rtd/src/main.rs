// main.rs —— rtd 入口: CLI 解析 / 生命周期 / 信号处理
//
// 线程/任务布局:
//   - 控制线程(std, 名字 rtd-control): 100Hz 实时闭环, clock_nanosleep TIMER_ABSTIME。
//   - REP 线程(std, 名字 rtd-rep): 阻塞 zmq REQ/REP, 改共享状态。
//   - PUB 任务(tokio): 50Hz 状态心跳 + 事件即时推送。
// 信号(SIGINT/SIGTERM)由 tokio 信号驱动捕获 -> 置 stop -> 控制线程零速/关串口 -> join。

mod core;
mod io;
mod kin;
mod odom;
mod pid;
mod proto;
mod util;
mod zmq;

// 注意: log_i!/log_e! 宏经 #[macro_export] 已在 crate 根作用域, main.rs 无需再 use
use crate::util::mono_now;
use std::sync::{Arc, Mutex};
use tokio::signal::unix::{SignalKind, signal};
use tokio::sync::mpsc;

struct Opts {
    port: String,
    tick_hz: u64,
    cmd_port: u16,
    pub_port: u16,
    simulate: bool,
    recv_budget_ms: f64,
}

fn usage(prog: &str) {
    eprintln!(
        "用法: {} [选项]\n\
         \x20 --port PATH      串口设备 (默认 /dev/ttyUSB0)\n\
         \x20 --tick-hz N      控制频率 Hz (默认 100)\n\
         \x20 --cmd-port N     ZMQ REP 端口 (默认 6010)\n\
         \x20 --pub-port N     ZMQ PUB 端口 (默认 6011)\n\
         \x20 --simulate       不开串口, 用虚拟设备联调(安全, 不碰真实硬件)\n\
         \x20 --recv-budget MS 每 tick 编码器应答接收预算 ms (默认 3)\n\
         \x20 --help           显示本帮助\n",
        prog
    );
}

fn parse_args() -> Opts {
    let mut opts = Opts {
        port: "/dev/ttyUSB0".to_string(),
        tick_hz: 100,
        cmd_port: 6010,
        pub_port: 6011,
        simulate: false,
        recv_budget_ms: 3.0,
    };
    let args: Vec<String> = std::env::args().collect();
    let prog = args.get(0).map(|s| s.as_str()).unwrap_or("rtd");
    let mut i = 1;
    let next_val = |i: &mut usize, args: &[String], name: &str| -> Option<String> {
        if *i + 1 < args.len() {
            *i += 1;
            Some(args[*i].clone())
        } else {
            eprintln!("{} 缺少参数值", name);
            None
        }
    };
    while i < args.len() {
        match args[i].as_str() {
            "--port" => match next_val(&mut i, &args, "--port") {
                Some(v) => opts.port = v,
                None => usage(prog),
            },
            "--tick-hz" => match next_val(&mut i, &args, "--tick-hz") {
                Some(v) => match v.parse::<u64>() {
                    Ok(n) if n >= 10 && n <= 1000 => opts.tick_hz = n,
                    _ => {
                        eprintln!("--tick-hz 取值 10..1000");
                        std::process::exit(2);
                    }
                },
                None => usage(prog),
            },
            "--cmd-port" => match next_val(&mut i, &args, "--cmd-port") {
                Some(v) => match v.parse::<u16>() {
                    Ok(n) => opts.cmd_port = n,
                    Err(_) => {
                        eprintln!("--cmd-port 必须是端口号");
                        std::process::exit(2);
                    }
                },
                None => usage(prog),
            },
            "--pub-port" => match next_val(&mut i, &args, "--pub-port") {
                Some(v) => match v.parse::<u16>() {
                    Ok(n) => opts.pub_port = n,
                    Err(_) => {
                        eprintln!("--pub-port 必须是端口号");
                        std::process::exit(2);
                    }
                },
                None => usage(prog),
            },
            "--recv-budget" => match next_val(&mut i, &args, "--recv-budget") {
                Some(v) => match v.parse::<f64>() {
                    Ok(n) if n >= 0.0 && n <= 100.0 => opts.recv_budget_ms = n,
                    _ => {
                        eprintln!("--recv-budget 取值 0..100");
                        std::process::exit(2);
                    }
                },
                None => usage(prog),
            },
            "--simulate" => opts.simulate = true,
            "--help" | "-h" => {
                usage(prog);
                std::process::exit(0);
            }
            other => {
                eprintln!("未知参数: {}", other);
                usage(prog);
                std::process::exit(2);
            }
        }
        i += 1;
    }
    opts
}

fn main() {
    let opts = parse_args();
    let started_at = mono_now();

    // 创建 IO(真实串口或 --simulate 虚拟设备; 失败则退出)
    let io = match io::create_io(opts.simulate, &opts.port, opts.tick_hz as f64) {
        Some(i) => Arc::new(Mutex::new(i)),
        None => {
            log_e!("IO 初始化失败 (port={})", opts.port);
            std::process::exit(1);
        }
    };

    let core = Arc::new(Mutex::new(core::Core::new()));

    // PUB 事件通道: 控制线程 push(非阻塞), PUB 任务 pop
    let (pub_tx, pub_rx) = mpsc::unbounded_channel();
    core.lock().unwrap().pub_tx = Some(pub_tx);

    let rt = match tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
    {
        Ok(r) => r,
        Err(e) => {
            log_e!("tokio runtime 创建失败: {}", e);
            std::process::exit(1);
        }
    };

    // 控制线程(专用 std 线程: 绝对节拍 + 阻塞串口 IO, 不放 async)
    let c_core = core.clone();
    let c_io = io.clone();
    let tick_hz = opts.tick_hz as f64;
    let recv_budget = opts.recv_budget_ms;
    let ctrl_thread = std::thread::Builder::new()
        .name("rtd-control".into())
        .spawn(move || core::control_loop(c_core, c_io, tick_hz, recv_budget))
        .expect("spawn rtd-control 失败");

    // REP 线程(专用 std 线程: 阻塞 zmq + 同步 frame 等待)
    let r_core = core.clone();
    let cmd_url = format!("tcp://127.0.0.1:{}", opts.cmd_port);
    let rep_thread = std::thread::Builder::new()
        .name("rtd-rep".into())
        .spawn(move || zmq::rep_loop(r_core, &cmd_url))
        .expect("spawn rtd-rep 失败");

    // PUB 任务(tokio: 50Hz 心跳 + 事件即时推送)
    let p_core = core.clone();
    let pub_url = format!("tcp://127.0.0.1:{}", opts.pub_port);
    rt.spawn(zmq::pub_loop(p_core, pub_url, pub_rx));

    zmq::startup_banner(
        opts.tick_hz,
        &opts.port,
        opts.cmd_port,
        opts.pub_port,
        opts.simulate,
        started_at,
    );

    // 等待 SIGINT / SIGTERM(tokio 信号驱动, 平滑停机)
    rt.block_on(async {
        let mut sigint = match signal(SignalKind::interrupt()) {
            Ok(s) => s,
            Err(e) => {
                log_e!("注册 SIGINT 失败: {}", e);
                return;
            }
        };
        let mut sigterm = match signal(SignalKind::terminate()) {
            Ok(s) => s,
            Err(e) => {
                log_e!("注册 SIGTERM 失败: {}", e);
                return;
            }
        };
        tokio::select! {
            _ = sigint.recv() => log_i!("收到 SIGINT"),
            _ = sigterm.recv() => log_i!("收到 SIGTERM"),
        }
    });

    log_i!("正在停机: 零速 -> 关串口");
    core.lock().unwrap().stop = true;
    let _ = ctrl_thread.join();
    let _ = rep_thread.join();
    log_i!("rtd 已退出");
}
