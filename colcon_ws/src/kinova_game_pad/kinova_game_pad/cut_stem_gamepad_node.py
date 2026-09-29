import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Joy

from plant_cutter_msgs.action import CutStem


class CutStemGamepadNode(Node):
    """Sends/cancels a CutStem action goal from the PS4 gamepad, while the
    enable (deadman) button is held:
        - 'X' (Cross) button press  -> send goal (start_cutting=True)
        - 'O' (Circle) button press -> cancel the active goal

    Button indices follow this workspace's PS4 x-pad mapping (see
    clearpath_control/config/generic/teleop_ps4.yaml): X=0, O=1, L1=4.
    """

    def __init__(self):
        super().__init__('cut_stem_gamepad_node')

        self.declare_parameter('joy_topic', 'joy_teleop/joy')
        self.declare_parameter('action_name', 'cut_stem')
        self.declare_parameter('start_button', 0)   # 'X' (Cross)
        self.declare_parameter('cancel_button', 1)  # 'O' (Circle)
        self.declare_parameter('enable_button', 4)  # L1 (deadman/enable)

        joy_topic = self.get_parameter('joy_topic').value
        action_name = self.get_parameter('action_name').value
        self._start_button = self.get_parameter('start_button').value
        self._cancel_button = self.get_parameter('cancel_button').value
        self._enable_button = self.get_parameter('enable_button').value

        self._prev_start_pressed = False
        self._prev_cancel_pressed = False
        self._goal_handle = None
        self._goal_pending = False

        cb_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self, CutStem, action_name, callback_group=cb_group)
        self._joy_sub = self.create_subscription(
            Joy, joy_topic, self._joy_callback, 10,
            callback_group=cb_group)

        self.get_logger().info(
            f"Listening on '{joy_topic}': action '{action_name}' is sent on "
            f"button {self._start_button} and cancelled on button "
            f"{self._cancel_button}, while enable button "
            f"{self._enable_button} is held.")

    def _joy_callback(self, msg: Joy):
        max_index = max(
            self._start_button, self._cancel_button, self._enable_button)
        if len(msg.buttons) <= max_index:
            return

        enabled = bool(msg.buttons[self._enable_button])
        start_pressed = bool(msg.buttons[self._start_button])
        cancel_pressed = bool(msg.buttons[self._cancel_button])

        # Trigger only on the rising edge so a held button does not
        # spam repeated goals/cancellations.
        if enabled and start_pressed and not self._prev_start_pressed:
            self._send_cut_stem_goal()
        if enabled and cancel_pressed and not self._prev_cancel_pressed:
            self._cancel_cut_stem_goal()

        self._prev_start_pressed = start_pressed and enabled
        self._prev_cancel_pressed = cancel_pressed and enabled

    def _send_cut_stem_goal(self):
        if self._goal_handle is not None or self._goal_pending:
            self.get_logger().warn(
                'CutStem goal already in progress, ignoring button press.')
            return

        if not self._action_client.server_is_ready():
            self.get_logger().warn(
                'CutStem action server is not available.')
            return

        goal_msg = CutStem.Goal()
        goal_msg.start_cutting = True

        self.get_logger().info('Sending CutStem goal (start_cutting=True).')
        self._goal_pending = True
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _cancel_cut_stem_goal(self):
        if self._goal_handle is None:
            self.get_logger().warn('No active CutStem goal to cancel.')
            return

        self.get_logger().info('Cancelling CutStem goal.')
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_response_callback)

    def _cancel_response_callback(self, future):
        if future.result().goals_canceling:
            self.get_logger().info('CutStem cancel request accepted.')
        else:
            self.get_logger().warn(
                'CutStem cancel request was rejected by the action server.')

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        self._goal_pending = False
        if not goal_handle.accepted:
            self.get_logger().warn('CutStem goal was rejected.')
            return

        self.get_logger().info('CutStem goal accepted.')
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f'CutStem feedback: state={feedback.current_state}, '
            f'distance_to_target={feedback.distance_to_target}')

    def _result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f'CutStem result: success={result.success}, '
            f'message="{result.message}"')
        self._goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = CutStemGamepadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
