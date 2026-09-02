#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String


class PersonZoneStatusMonitor(Node):
    """Print a compact combined view of the person-zone status topics."""

    def __init__(self) -> None:
        super().__init__("person_zone_status_monitor")
        self.zone = "NONE"
        self.warning = False
        self.exit_direction = "NONE"
        self.edge_warning = "NONE"
        self.diagnostics = "STARTING"
        self.pose_device = "starting"
        self.received = {"zone": False, "warning": False, "exit": False}
        self.last_printed = None

        self.create_subscription(
            String, "/person_zone_state", self.zone_callback, 10
        )
        self.create_subscription(
            Bool, "/person_emergency", self.warning_callback, 10
        )
        self.create_subscription(
            String, "/person_exit_direction", self.exit_callback, 10
        )
        self.create_subscription(
            String, "/person_edge_warning", self.edge_callback, 10
        )
        self.create_subscription(
            String, "/person_zone/diagnostics", self.diagnostics_callback, 10
        )
        self.create_subscription(
            String, "/person_pose/device", self.pose_device_callback, 10
        )
        self.create_subscription(
            String, "/person_exit_event", self.event_callback, 10
        )
        self.initial_timer = self.create_timer(0.5, self.print_initial_status)

    def print_initial_status(self) -> None:
        self.initial_timer.cancel()
        self.received["zone"] = True
        self.received["warning"] = True
        self.print_status()

    def zone_callback(self, msg: String) -> None:
        self.zone = msg.data
        self.received["zone"] = True
        self.print_status()

    def warning_callback(self, msg: Bool) -> None:
        self.warning = msg.data
        self.received["warning"] = True
        self.print_status()

    def exit_callback(self, msg: String) -> None:
        self.exit_direction = msg.data
        self.received["exit"] = True
        self.print_status()

    def edge_callback(self, msg: String) -> None:
        self.edge_warning = msg.data
        self.print_status()

    def diagnostics_callback(self, msg: String) -> None:
        self.diagnostics = msg.data
        self.print_status()

    def pose_device_callback(self, msg: String) -> None:
        self.pose_device = msg.data
        self.print_status()

    @staticmethod
    def event_callback(msg: String) -> None:
        print(f"Exit Event : {msg.data}\n---", flush=True)

    def print_status(self) -> None:
        # Exit is an event topic and may not publish until a detection cycle.
        # Zone and warning are continuous, so they are sufficient for an
        # initial status line; Exit remains NONE until an event arrives.
        if not self.received["zone"] or not self.received["warning"]:
            return
        status = (
            self.zone,
            self.warning,
            self.exit_direction,
            self.edge_warning,
            self.diagnostics,
            self.pose_device,
        )
        if status == self.last_printed:
            return
        self.last_printed = status
        print(
            f"Zone : {self.zone}\n"
            f"Warning : {'TRUE' if self.warning else 'FALSE'}\n"
            f"Exit : {self.exit_direction}\n"
            f"Edge : {self.edge_warning}\n"
            f"Diagnostics : {self.diagnostics}\n"
            f"Pose device : {self.pose_device}\n"
            "---",
            flush=True,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PersonZoneStatusMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
