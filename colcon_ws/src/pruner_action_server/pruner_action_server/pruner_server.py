import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import serial
import time

from serial_interfaces.action import SendInteger

class PrunerActionServer(Node):
    def __init__(self):
        super().__init__('pruner_action_server')
        
        self.declare_parameter('port', '/dev/ttyOpenCR')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout_sec', 30.0) 
        
        serial_port = self.get_parameter('port').value
        baud_rate = self.get_parameter('baudrate').value

        self.ser = None
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
            self.get_logger().info(f'Successfully opened serial port {serial_port} at {baud_rate} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')

        self._action_server = ActionServer(
            self,
            SendInteger,
            'pruner_action_server',
            self.execute_callback
        )
        self.get_logger().info('Action server "pruner_action_server" is ready.')

    def execute_callback(self, goal_handle):
        target_val = goal_handle.request.target_integer
        self.get_logger().info(f'Received goal to send integer: {target_val}')
        
        feedback_msg = SendInteger.Feedback()
        result = SendInteger.Result()

        if self.ser is None or not self.ser.is_open:
            self.get_logger().error('Cannot send data. Serial port is not open.')
            goal_handle.abort()
            result.success = False
            result.message = "Serial port unavailable."
            return result

        try:
            self.ser.reset_input_buffer()

            feedback_msg.status = f"Preparing to send {target_val} over serial..."
            goal_handle.publish_feedback(feedback_msg)

            command = f"{target_val}\n"
            self.ser.write(command.encode('utf-8'))
            self.ser.flush()
            self.get_logger().info('Command sent. Waiting for hardware completion feedback...')
            
            timeout = self.get_parameter('timeout_sec').value
            start_time = time.time()
            is_done = False
            is_failed = False
            
            # Listening loop: Monitors serial stream for DONE, FAIL, or Timeout
            while rclpy.ok() and (time.time() - start_time) < timeout:
                if self.ser.in_waiting > 0:
                    try:
                        line = self.ser.readline().decode('utf-8').strip()
                        if line:
                            self.get_logger().debug(f'Received from Arduino: {line}')
                            
                        if line == "STATUS:DONE":
                            is_done = True
                            break
                        elif line == "STATUS:FAIL":
                            is_failed = True
                            break
                    except UnicodeDecodeError:
                        self.get_logger().warn('Received malformed serial data.')
                
                feedback_msg.status = "Executing physical pruning cycle..."
                goal_handle.publish_feedback(feedback_msg)
                
                time.sleep(0.05)
            
            if is_failed:
                self.get_logger().error('Pruning cycle failed on hardware (STATUS:FAIL). Aborting goal.')
                goal_handle.abort()
                result.success = False
                result.message = "Pruner hardware reported STATUS:FAIL."
            elif is_done:
                goal_handle.succeed()
                result.success = True
                result.message = f"Successfully completed pruning cycle for command {target_val}"
                self.get_logger().info('Goal succeeded. Pruning confirmed.')
            else:
                self.get_logger().error('Pruning cycle timed out waiting for feedback from Arduino.')
                goal_handle.abort()
                result.success = False
                result.message = "Pruner action timed out waiting for hardware feedback."

        except Exception as e:
            self.get_logger().error(f'Error writing to/reading from serial port: {e}')
            goal_handle.abort()
            result.success = False
            result.message = str(e)
            
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = PrunerActionServer()
    
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        if action_server.ser and action_server.ser.is_open:
            action_server.ser.close()
            action_server.get_logger().info('Serial port closed.')
        action_server.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()