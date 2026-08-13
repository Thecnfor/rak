# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

There is no project packaging file, dependency lockfile, Makefile, or repository test suite. The documented runtime environment is Python 3.8+ on the Jetson Nano/WhalesBot hardware platform with PaddlePaddle Inference and the WhalesBot SDK available.

- Run the complete competition flow on the car: `python car_start_2026.py`
- Run the dual-camera teleoperation/data collection mode: `python -m smartcar.whalesbot.tools.collect_control`
- Run a syntax-only check without initializing hardware: `python -m compileall -q car_start_2026.py car_task_function.py car_wrap_2026.py collect_data.py smartcar`
- There is no configured lint command or automated test command. Individual task checks are performed by editing `main()` in `car_start_2026.py` to leave only the task under test, then running the script on hardware; `car_wrap_2026.py` also contains an interactive `manage()`/OCR debug entry point when run directly.

Do not expect the main scripts to run on a normal development machine: importing/initializing `MyCar` opens cameras, hardware controllers, background key handling, ZMQ inference clients, and Ernie clients.

## Architecture

The repository is a hardware-first Python application for a Baidu SmartCar agriculture competition robot.

- `car_start_2026.py` is the top-level orchestrator. Its `main()` initializes the robot and executes the ordered flow: seeding, animal detection, watering, shooting, harvesting, sorting/storage, order recognition, and delivery. `auto_lane_tracing()` is intentionally left as a commented lane-following test hook.
- `car_task_function.py` contains the competition choreography. Tasks share a module-global `my_car`, which is created by `init()` as `MyCar`; task functions combine odometry, lane-following distance moves, visual alignment, arm poses, suction, storage servo control, shooting, OCR, and order parsing. Preserve the task ordering and calibrated distances/poses unless deliberately recalibrating the field behavior.
- `car_wrap_2026.py` defines `MyCar(MecanumDriver)`, the main hardware/application façade. Construction initializes WhalesBot sensors and actuators, cameras, PID controllers, ZMQ inference clients, Ernie wrappers, streaming, and a daemon key thread. High-level motion methods (`lane_time`, `lane_dis`, `lane_dis_offset`, `move_to_position`, `move_to_detection_target`) sit above low-level mecanum/arm drivers.
- `smartcar/whalesbot/` is the hardware layer: vehicle drivers/controllers, arm control, serial/MC602 communication, cameras, streaming, PID and utility classes. `smartcar/__init__.py` re-exports the commonly used hardware and utility APIs.
- `smartcar/paddlebaidu/` is the perception and language layer. `infer_cs` provides the client interface used by `MyCar`; configured inference backends/models cover lane detection, task/front object detection, and OCR. `ernie_bot` wraps image/order analysis and prompt handling. Model assets live under `smartcar/paddlebaidu/models/`.
- `config_car.yml` is runtime configuration, not packaging metadata. It defines camera channels (`front: 1`, `side: 2`), IO pins, speed limits, lane/detection/location PID settings, ZMQ inference services (ports 5001–5004), model directories, and the Ernie access-token field. Hardware calibration and model/config changes can alter physical behavior.
- `collect_data.py` is a standalone data-collection entry point using two cameras and `CollectControlCar`; it writes lane/object images under the ignored `dataset/` directory. The lower-level control module is also runnable via the documented module command above.

## Runtime and calibration notes

- The expected hardware is Jetson Nano + MC602 controller + WhalesBot mecanum chassis, arm, cameras, storage rack, buzzer, and shooting mechanism. Camera indices and all movement distances are hardware/field calibrated.
- Detection results use the list shape `[cls_id, obj_id, label, score, x_c, y_c, w, h]` with normalized bounding-box coordinates. Task labels and their semantics are documented in `README.md`; changing labels requires corresponding task logic updates.
- `MyCar` starts inference clients and a key-monitoring thread during initialization. Use its `close()` path when writing standalone diagnostics so cameras, streaming, and the key thread are released.
- Keep secrets and local datasets out of commits. `.gitignore` excludes `dataset/`, virtual environments, caches, and access-token YAML files; `config_car.yml` currently contains a placeholder token field and should be treated as deployment configuration.
