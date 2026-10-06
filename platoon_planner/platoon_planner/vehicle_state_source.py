#!/usr/bin/env python3

"""
One-shot VehicleState reader for planner initialization.

Used by the interactive coverage planner to replace temporary
hardcoded R1 start coordinates with the real live vehicle state.
"""

import time

import rclpy
from rclpy.node import Node

from platoon_interfaces.msg import VehicleState


def get_vehicle_state_once(
    topic="/r1/vehicle_state",
    timeout_sec=30.0,
):
    """
    Wait for one VehicleState message and return:

        (north, east, body_yaw)

    VehicleState already uses the project's world_ned convention:

        x = North
        y = East
        body_yaw:
            0 = North
            +pi/2 = East
    """

    if timeout_sec <= 0.0:
        raise ValueError(
            "timeout_sec must be > 0"
        )

    initialized_here = False

    if not rclpy.ok():
        rclpy.init()
        initialized_here = True

    node = Node(
        "planner_vehicle_state_reader"
    )

    received = {
        "message": None
    }

    def callback(message):
        received["message"] = message

    subscription = node.create_subscription(
        VehicleState,
        topic,
        callback,
        10,
    )

    start_time = time.monotonic()

    try:

        while (
            received["message"] is None
            and
            time.monotonic()
            - start_time
            < timeout_sec
        ):

            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

        if received["message"] is None:

            raise RuntimeError(
                f"No VehicleState received from "
                f"{topic} within "
                f"{timeout_sec:.1f} seconds."
            )

        message = received[
            "message"
        ]

        return (
            float(message.x),
            float(message.y),
            float(message.body_yaw),
        )

    finally:

        # Keep a reference until after spinning is finished.
        del subscription

        node.destroy_node()

        if initialized_here and rclpy.ok():
            rclpy.shutdown()
