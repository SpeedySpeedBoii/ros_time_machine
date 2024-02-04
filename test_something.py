from typing import List

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor, TimeoutException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Duration, Time
from rclpy.timer import Timer
from rosgraph_msgs.msg import Clock

ClockType = _rclpy.ClockType


class Something:
    def __init__(self, node: Node):
        self._node = node
        self._first_timer = node.create_timer(2, self._first_cb)
        self._second_timer = None
        self._third_timer = None
        self.achievement_list = []

    def _first_cb(self) -> None:
        self._node.get_logger().info("first cb")
        self.achievement_list.append(1)
        self._first_timer.cancel()
        self._second_timer = self._node.create_timer(3, self._second_cb)

    def _second_cb(self) -> None:
        self._node.get_logger().info("second cb")
        self.achievement_list.append(2)
        self._second_timer.cancel()
        self._third_timer = self._node.create_timer(4, self._third_cb)

    def _third_cb(self) -> None:
        self._node.get_logger().info("third cb")
        self.achievement_list.append(3)
        self._third_timer.cancel()


@pytest.fixture
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def executor() -> SingleThreadedExecutor:
    return SingleThreadedExecutor()


@pytest.fixture
def testing_node(rclpy_init, executor: SingleThreadedExecutor):
    node = Node("test_node")
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    executor.add_node(node)
    yield node
    node.destroy_node()


def _duration(seconds: float) -> Duration:
    seconds_int = int(seconds)
    nanoseconds_int = (seconds - seconds_int) * 1000000000
    return Duration(seconds=int(seconds), nanoseconds=nanoseconds_int)


def _time(seconds: float) -> Time:
    seconds_int = int(seconds)
    nanoseconds_int = (seconds - seconds_int) * 1000000000
    return Time(
        seconds=int(seconds), nanoseconds=nanoseconds_int, clock_type=ClockType.ROS_TIME
    )


class TimeMachine:
    def __init__(self, node: Node):
        self._node = node
        self._clock_publisher = node.create_publisher(Clock, "/clock", 10)

    def speed_foreward(self, seconds: float) -> None:
        target_time = self._time_now() + _duration(seconds)
        self._speed_foreward_absolute(target_time)

    def speed_foreward_absolute(self, target_time: float) -> None:
        self._speed_foreward_absolute(_time(target_time))

    def _speed_foreward_absolute(self, target_time: Time) -> None:
        while (len(list(self._active_timers())) > 0) and (
            self._get_time_of_next_timer_trigger() <= target_time
        ):
            self._update_ros_time(self._get_time_of_next_timer_trigger())
            self._execute_pending_tasks()
        self._update_ros_time(target_time)

    def _time_now(self) -> Time:
        return self._node.get_clock().now()

    def _get_time_of_next_timer_trigger(self) -> Time:
        return self._node.get_clock().now() + Duration(
            nanoseconds=self._timers_sorted_by_proximity()[0].time_until_next_call()
        )

    def _active_timers(self) -> List[Timer]:
        return [timer for timer in self._node.timers if not timer.is_canceled()]

    def _timers_sorted_by_proximity(self) -> List[Timer]:
        return sorted(
            self._active_timers(), key=lambda timer: timer.time_until_next_call()
        )

    def _update_ros_time(self, time: Time) -> None:
        self._clock_publisher.publish(Clock(clock=time.to_msg()))

    def _execute_pending_tasks(self):
        while True:
            try:
                handler, _, _ = self._node.executor.wait_for_ready_callbacks(
                    timeout_sec=0
                )
                handler()
                if handler.exception() is not None:
                    raise handler.exception()
            except TimeoutException:
                break


@pytest.fixture
def time_machine(testing_node: Node):
    return TimeMachine(testing_node)


# @pytest.mark.parametrize("execution_number", range(1000))
def test_something(testing_node, time_machine):
    something = Something(testing_node)

    time_machine.speed_foreward_absolute(1.9)
    assert not something.achievement_list

    time_machine.speed_foreward_absolute(2)
    assert something.achievement_list == [1]

    time_machine.speed_foreward_absolute(2 + 2.9)
    assert something.achievement_list == [1]

    time_machine.speed_foreward_absolute(2 + 3)
    assert something.achievement_list == [1, 2]

    time_machine.speed_foreward_absolute(2 + 3 + 3.9)
    assert something.achievement_list == [1, 2]

    time_machine.speed_foreward_absolute(2 + 3 + 4)
    assert something.achievement_list == [1, 2, 3]


# @pytest.mark.parametrize("execution_number", range(1000))
# def test_something2(testing_node, time_machine, execution_number):
#     something = Something(testing_node)

#     assert not something.achievement_list

#     something.achievement_list.append(1)
#     assert something.achievement_list == [1]

#     assert something.achievement_list == [1]

#     something.achievement_list.append(2)
#     assert something.achievement_list == [1, 2]

#     assert something.achievement_list == [1, 2]

#     something.achievement_list.append(3)
#     assert something.achievement_list == [1, 2, 3]
