import rclpy
import time
from rclpy.action import ActionClient
from rclpy.node import Node
from ur_script_generator_msgs.srv import PlanToTarget
from ur_script_msgs.action import ExecuteScript

class URScriptActionDummy(Node):
    def __init__(self):
        super().__init__("ur_script_action_dummy")

        self.ursg_client = self.create_client(PlanToTarget, "/ur_script_generator")
        self.ursg_request = PlanToTarget.Request()

        while not self.ursg_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    def send_request(self):
        self.ursg_request.robot_id = "ursim"
        self.ursg_request.command = "move"
        self.ursg_request.move_type = "move_j"
        self.ursg_request.velocity = 0.5
        self.ursg_request.acceleration = 0.3
        self.ursg_request.goal_feature_name = "frame_4"
        self.ursg_request.tcp_name = "tool_0"
        self.future = self.ursg_client.call_async(self.ursg_request)

    def run(self):
        time.sleep(2)
        self.send_request()

        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                except Exception as e:
                    self.get_logger().info("Service call failed %r" % (e,))
                else:
                    ursg_response = response.ur_script
                    self.get_logger().info(
                        "Result of generate_ur_script: %s" % ursg_response
                    )

                    self.action_client = ActionClient(self, ExecuteScript, "/ur_script")

                    def send_goal(script):
                        goal_msg = ExecuteScript.Goal()
                        goal_msg.script = script
                        self.action_client.wait_for_server()
                        return self.action_client.send_goal_async(goal_msg)

                    future = send_goal(ursg_response)
                    rclpy.spin_until_future_complete(self.action_client, future)
                break

def main(args=None):
    rclpy.init(args=args)

    ursg_client = URScriptActionDummy()
    ursg_client.run()

    ursg_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()