# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

There is no project packaging file, dependency lockfile, Makefile, or repository test suite. The documented runtime environment is Python 3.8+ on the Jetson Nano/WhalesBot hardware platform with PaddlePaddle Inference and the WhalesBot SDK available.

- Run the complete competition flow on the car: `python run.py all` (单任务如 `python run.py seeding`; `python run.py --help` 查看全部参数)
- Run the dual-camera teleoperation/data collection mode: `python -m smartcar.whalesbot.tools.collect_control`
- Run a syntax-only check without initializing hardware: `python -m compileall -q run.py tasks collect_data.py smartcar`
- There is no configured lint command or automated test command. Individual task checks are performed by running the corresponding task via `run.py` on hardware.

Do not expect the main scripts to run on a normal development machine: importing/initializing `MyCar` opens cameras, hardware controllers, background key handling, ZMQ inference clients, and Ernie clients.

## Architecture

The repository is a hardware-first Python application for a Baidu SmartCar agriculture competition robot.

- `tasks/` is the task layer (refactored from the former `car_start_2026.py` / `car_task_function.py` / `car_wrap_2026.py` monolith, which no longer exist). `tasks/tools/car.py` defines `MyCar(MotionMixin, PerceptionMixin, MecanumDriver)`, the main hardware/application façade; `tasks/tools/motion.py` / `perception.py` / `pids.py` / `helpers.py` provide the individual capabilities. `tasks/*.py` (seeding, target_detection, watering, shooting, harvesting, sorting, ordering, delivery) are the competition tasks, orchestrated by `run.py`. Preserve the task ordering and calibrated distances/poses unless deliberately recalibrating the field behavior.
- `smartcar/whalesbot/` is the hardware layer: vehicle drivers/controllers, arm control, serial/MC602 communication, cameras, streaming, PID and utility classes. `smartcar/__init__.py` re-exports the commonly used hardware and utility APIs.
- `smartcar/paddlebaidu/` is the perception and language layer. `infer_cs` provides the client interface used by `MyCar`; configured inference backends/models cover lane detection, task/front object detection, and OCR. `ernie_bot` wraps image/order analysis and prompt handling. Model assets live under `smartcar/paddlebaidu/models/`.
- `config_car.yml` is runtime configuration, not packaging metadata. It defines camera channels (`front: 1`, `side: 2`), IO pins, speed limits, lane/detection/location PID settings, ZMQ inference services (ports 5001–5004), model directories, and the Ernie access-token field. Hardware calibration and model/config changes can alter physical behavior.
- `collect_data.py` is a standalone data-collection entry point using two cameras and `CollectControlCar`; it writes lane/object images under the ignored `dataset/` directory. The lower-level control module is also runnable via the documented module command above.

## Runtime and calibration notes

- The expected hardware is Jetson Nano + MC602 controller + WhalesBot mecanum chassis, arm, cameras, storage rack, buzzer, and shooting mechanism. Camera indices and all movement distances are hardware/field calibrated.
- Detection results use the list shape `[cls_id, obj_id, label, score, x_c, y_c, w, h]` with normalized bounding-box coordinates. Task labels and their semantics are documented in `README.md`; changing labels requires corresponding task logic updates.
- `MyCar` starts inference clients and a key-monitoring thread during initialization. Use its `close()` path when writing standalone diagnostics so cameras, streaming, and the key thread are released.
- Keep secrets and local datasets out of commits. `.gitignore` excludes `dataset/`, virtual environments, caches, and access-token YAML files; `config_car.yml` currently contains a placeholder token field and should be treated as deployment configuration.
