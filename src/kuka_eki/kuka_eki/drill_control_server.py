"""Spindle on/off service.

Exposes ``drill_on`` (std_srvs/Trigger). One call energises the spindle,
waits ``spindle_on_duration`` seconds, then switches it off again.
"""

import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from kuka_eki.tcp_client import TcpClient

MODE_ON = "<RobotMode><Mode>1</Mode></RobotMode>"
MODE_OFF = "<RobotMode><Mode>0</Mode></RobotMode>"


class DrillControlServer(Node):
    def __init__(self):
        super().__init__('drill_control_server')

        self.declare_parameter('drill_ip', '172.31.1.147')
        self.declare_parameter('drill_port', 54603)
        self.declare_parameter('spindle_on_duration', 2.0)
        self.declare_parameter('socket_timeout', 5.0)

        self.srv = self.create_service(Trigger, 'drill_on', self.drill_on_callback)
        self.get_logger().info(
            f"drill_on service ready "
            f"(target {self.get_parameter('drill_ip').value}:"
            f"{self.get_parameter('drill_port').value})"
        )

    def drill_on_callback(self, request, response):
        del request  # Trigger carries no fields

        ip = self.get_parameter('drill_ip').value
        port = int(self.get_parameter('drill_port').value)
        duration = float(self.get_parameter('spindle_on_duration').value)
        timeout = float(self.get_parameter('socket_timeout').value)

        client = None
        try:
            client = TcpClient((ip, port), timeout=timeout)
            client.connect()
            self.get_logger().info(f"Connected to spindle controller {ip}:{port}")

            client.sendall(MODE_ON.encode('utf-8'))
            try:
                reply = client.recv(1024).decode('utf-8', errors='replace')
                self.get_logger().info(f"Spindle ON acknowledged: {reply}")
            except OSError:
                # Some controller firmware does not acknowledge; not fatal.
                self.get_logger().warn("Spindle ON sent but no acknowledgement received.")

            time.sleep(duration)

            client.sendall(MODE_OFF.encode('utf-8'))
            self.get_logger().info(f"Spindle OFF sent after {duration:.1f}s")

            response.success = True
            response.message = f"Spindle cycled on for {duration:.1f}s and switched off."

        except Exception as exc:  # noqa: BLE001 - report every failure to the caller
            self.get_logger().error(f"Spindle control failed: {exc}")
            response.success = False
            response.message = f"Spindle control failed: {exc}"

        finally:
            # `client` may be None if the constructor itself raised — guard it.
            if client is not None:
                client.close()
                self.get_logger().info("Spindle connection closed.")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DrillControlServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
