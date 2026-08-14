#!/usr/bin/env python3
"""Run the complete competition flow or one task on the real car."""

import argparse

from tasks import delivery, harvesting, ordering, seeding, shooting, sorting
from tasks import target_detection, watering


def run_all(car):
    seeding.run(car)
    animals = target_detection.run(car)
    watering.run(car)
    shooting.run(car)
    harvesting.run(car)
    sorting.run(car)
    orders = ordering.run(car)
    delivery.run(car)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "task",
        nargs="?",
        default="all",
        choices=[
            "all", "seeding", "target-detection", "watering", "shooting",
            "harvesting", "sorting", "ordering", "delivery",
        ],
    )
    parser.add_argument("--no-reset", action="store_true", help="skip arm and odometry reset")
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
