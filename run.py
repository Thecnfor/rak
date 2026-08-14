#!/usr/bin/env python3
"""Run the complete competition flow or one task on the real car."""

import argparse

from tasks import delivery, harvesting, ordering, seeding, shooting, sorting
from tasks import target_detection, watering


# 任务名 -> 执行函数（参数名与比赛顺序保持一致）
RUNNERS = {
    "seeding": seeding.run,
    "target_detection": target_detection.run,
    "watering": watering.run,
    "shooting": shooting.run,
    "harvesting": harvesting.run,
    "sorting": sorting.run,
    "ordering": ordering.run,
    "delivery": delivery.run,
}


def run_all(car):
    """
    按键驱动的完整比赛流程。

    使用车载按键编排（见 tasks/orchestrator.py）：
        按键 4 = 一键启动 / 重来（重来只跑未完成的任务）
        按键 1 = 跳过当前任务（不标记完成，下次重来仍可补做）
        按键 3 = 急停（终止本轮，不标记完成；重来从头再跑）
    已完成任务仅记录在内存中，重启程序即全新一次。
    """
    from tasks.orchestrator import Orchestrator

    orch = Orchestrator(car)
    try:
        while True:
            # 等待一键启动（按 4），重来则只跑未完成的任务
            orch.wait_start()
            for task_name in orch.schedule():
                print(f"=== 开始任务: {task_name} ===")
                # 任务执行期间监听跳过（按 1）/急停（按 3）按键
                orch.start_skip_listener()
                try:
                    RUNNERS[task_name](car)
                finally:
                    skipped, emergency = orch.stop_skip_listener()
                if emergency:
                    print("=== 急停，本轮终止（重来请按 4） ===")
                    break
                if skipped:
                    print(f"=== 已跳过任务: {task_name}（不标记完成，下次可补做） ===")
                    continue
                orch.mark_done(task_name)
            if not orch.schedule():
                print("所有任务均已完成，流程结束")
                break
    finally:
        orch.abort()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "task",
        nargs="?",
        default="all",
        choices=[
            "all",
            "seeding",
            "target-detection",
            "watering",
            "shooting",
            "harvesting",
            "sorting",
            "ordering",
            "delivery",
        ],
    )
    parser.add_argument(
        "--no-reset", action="store_true", help="skip arm and odometry reset"
    )
    args = parser.parse_args()

    from tasks.tools import create_car

    car = create_car(reset=not args.no_reset)
    try:
        if args.task == "all":
            run_all(car)
        elif args.task == "seeding":
            seeding.run(car)
        elif args.task == "target-detection":
            print(target_detection.run(car))
        elif args.task == "watering":
            watering.run(car)
        elif args.task == "shooting":
            shooting.run(car)
        elif args.task == "harvesting":
            harvesting.run(car)
        elif args.task == "sorting":
            sorting.run(car)
        elif args.task == "ordering":
            print(ordering.run(car))
        elif args.task == "delivery":
            delivery.run(car)
    finally:
        car.stop()
        car.close()


if __name__ == "__main__":
    main()
