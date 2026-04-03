from __future__ import annotations

import importlib
import sys
import types
import unittest
from pathlib import Path


class FakePublisher:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.published: list[object] = []

    def publish(self, msg: object) -> None:
        self.published.append(msg)


class FakeLogger:
    def error(self, _msg: str) -> None:
        pass


class FakeFuture:
    def __init__(self) -> None:
        self._done = False
        self._callbacks: list = []

    def add_done_callback(self, callback) -> None:
        self._callbacks.append(callback)

    def done(self) -> bool:
        return self._done

    def result(self):
        return None


class FakeClient:
    def wait_for_service(self, timeout_sec=None) -> bool:
        return True

    def call_async(self, _request) -> FakeFuture:
        return FakeFuture()


class FakeNode:
    def __init__(self) -> None:
        self.publishers: dict[str, FakePublisher] = {}
        self.logger = FakeLogger()

    def create_publisher(self, _msg_type, topic: str, _qos: int) -> FakePublisher:
        publisher = FakePublisher(topic)
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, *_args, **_kwargs):
        return None

    def create_client(self, *_args, **_kwargs) -> FakeClient:
        return FakeClient()

    def get_logger(self) -> FakeLogger:
        return self.logger


def _install_fake_robot_dependencies() -> None:
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    rclpy.node = rclpy_node
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node

    bridge_interfaces = types.ModuleType("bridge_interfaces")
    bridge_interfaces_msg = types.ModuleType("bridge_interfaces.msg")
    bridge_interfaces_srv = types.ModuleType("bridge_interfaces.srv")

    def make_message(name: str):
        return type(name, (), {})

    for name in [
        "DCEnable",
        "DCHome",
        "DCPidReq",
        "DCPidSet",
        "DCResetPosition",
        "DCSetPosition",
        "DCSetPwm",
        "DCSetVelocity",
        "DCStateAll",
        "IOInputState",
        "IOSetLed",
        "IOSetNeopixel",
        "SensorImu",
        "SensorKinematics",
        "ServoEnable",
        "ServoSet",
        "ServoStateAll",
        "StepConfigSet",
        "StepEnable",
        "StepHome",
        "StepMove",
        "StepStateAll",
        "SysOdomReset",
        "SystemPower",
        "SystemState",
    ]:
        setattr(bridge_interfaces_msg, name, make_message(name))

    class SetFirmwareState:
        class Request:
            def __init__(self) -> None:
                self.target_state = 0
                self.timeout_sec = 0.0

    bridge_interfaces_srv.SetFirmwareState = SetFirmwareState
    bridge_interfaces.msg = bridge_interfaces_msg
    bridge_interfaces.srv = bridge_interfaces_srv
    sys.modules["bridge_interfaces"] = bridge_interfaces
    sys.modules["bridge_interfaces.msg"] = bridge_interfaces_msg
    sys.modules["bridge_interfaces.srv"] = bridge_interfaces_srv


class RobotApiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        _install_fake_robot_dependencies()
        package_root = Path(__file__).resolve().parents[1]
        if str(package_root) not in sys.path:
            sys.path.insert(0, str(package_root))
        cls.robot_module = importlib.import_module("robot.robot")

    def setUp(self) -> None:
        self.node = FakeNode()
        self.robot = self.robot_module.Robot(self.node)

    def test_diff_drive_uses_1_based_motor_numbers(self) -> None:
        self.robot.set_velocity(100.0, 0.0)

        published = self.node.publishers["/dc_set_velocity"].published
        self.assertEqual(len(published), 2)
        self.assertEqual([msg.motor_number for msg in published], [1, 2])

    def test_diff_drive_mapping_is_configurable(self) -> None:
        self.robot.set_drive_wheels(3, 4)
        self.robot.set_velocity(100.0, 0.0)

        published = self.node.publishers["/dc_set_velocity"].published
        self.assertEqual([msg.motor_number for msg in published], [3, 4])

    def test_zero_brightness_defaults_to_led_off_mode(self) -> None:
        self.robot.set_led(1, 0)
        msg = self.node.publishers["/io_set_led"].published[-1]
        self.assertEqual(msg.led_id, 1)
        self.assertEqual(msg.brightness, 0)
        self.assertEqual(msg.mode, 0)

    def test_nonzero_brightness_defaults_to_steady_mode(self) -> None:
        self.robot.set_led(1, 255)
        msg = self.node.publishers["/io_set_led"].published[-1]
        self.assertEqual(msg.mode, 1)
        self.assertEqual(msg.brightness, 255)

    def test_invalid_motor_number_fails_fast(self) -> None:
        with self.assertRaisesRegex(ValueError, "motor_id must be between 1 and 4"):
            self.robot.set_motor_velocity(0, 100.0)

    def test_button_edges_are_latched_in_subscription_callback(self) -> None:
        first = self.robot_module.IOInputState()
        first.button_mask = 0
        first.limit_mask = 0
        self.robot._on_io_input(first)

        pressed = self.robot_module.IOInputState()
        pressed.button_mask = 0b1
        pressed.limit_mask = 0
        self.robot._on_io_input(pressed)

        self.assertTrue(self.robot.get_button(1))
        self.assertTrue(self.robot.was_button_pressed(1))
        self.assertFalse(self.robot.was_button_pressed(1))


if __name__ == "__main__":
    unittest.main()
