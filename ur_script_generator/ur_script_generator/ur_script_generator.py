import os
import json
import math
import rclpy
from rclpy.node import Node
from ur_script_generator_msgs.srv import GenerateURScript
from tf_tools_msgs.srv import LookupTransform

BASEFRAME_ID = "base"
FACEPLATE_ID = "tool0"


class URScriptGenerator(Node):
    def __init__(self):
        super().__init__("ur_script_generator")
        self.srv = self.create_service(
            GenerateURScript, "ur_script_generator", self.ur_script_generator_callback
        )

        self.scene_parameters_path = self.declare_parameter(
            "scene_parameters_path", "default value"
        )
        self.scene_parameters_path = (
            self.get_parameter("scene_parameters_path")
            .get_parameter_value()
            .string_value
        )

        self.driver_parameters = self.declare_parameter(
            "driver_parameters", "default value"
        )

        self.driver_parameters = (
            self.get_parameter("driver_parameters").get_parameter_value().string_value
        )

        self.client = self.create_client(LookupTransform, "tf_lookup")
        self.request = LookupTransform.Request()
        self.response = LookupTransform.Response()

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("tf lookup service not available, waiting again...")

        self.have_preferred_joint_configuration = False

    def ur_script_generator_callback(self, request, response):
        self.get_logger().info("Got request '{asdf}'".format(asdf=request))
        move_j = request.command == "move" and request.move_type == "move_j"
        move_l = request.command == "move" and request.move_type == "move_l"
        freedrive_mode = request.command == "freedrive_mode"
        end_freedrive_mode = request.command == "end_freedrive_mode"
        if move_j:
            response.ur_script = self.generate_move_j(request)
            return response
        elif move_l:
            response.ur_script = self.generate_move_l(request)
            return response
        elif freedrive_mode:
            response.ur_script = self.freedrive_mode(request)
            return response
        elif end_freedrive_mode:
            response.ur_script = self.end_freedrive_mode()
            return response
        else:
            self.get_logger().warn("Unknown request '{asdf}'".format(asdf=request))
            pass

    def tf_lookup(self, parent, child, deadline):
        self.request.parent_id = parent
        self.request.child_id = child
        self.request.deadline = deadline
        self.future = self.client.call_async(self.request)
        self.get_logger().info(f"TF lookup request sent: {self.request}.")
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                except Exception as e:
                    self.get_logger().error(
                        f"TF lookup service call failed with: {(e,)}."
                    )
                else:
                    self.response = response
                    self.get_logger().info(f"TF lookup result: {self.response}.")
                finally:
                    self.get_logger().info(f"TF lookup service call completed.")
                    return response

    def pose_to_string(self, tf_stamped):
        x = tf_stamped.transform.translation.x
        y = tf_stamped.transform.translation.y
        z = tf_stamped.transform.translation.z
        q = tf_stamped.transform.rotation
        angle = 2 * math.acos(q.w)
        den = math.sqrt(1 - q.w * q.w)
        if den < 0.001:
            Rx = q.x * angle
            Ry = q.y * angle
            Rz = q.z * angle
        else:
            Rx = (q.x / den) * angle
            Ry = (q.y / den) * angle
            Rz = (q.z / den) * angle

        return (
            "p["
            + str(x)
            + ", "
            + str(y)
            + ", "
            + str(z)
            + ", "
            + str(Rx)
            + ", "
            + str(Ry)
            + ", "
            + str(Rz)
            + "]"
        )

    def generate_move_j(self, request):
        self.get_logger().info(
            "Requested move_j to: %s with tool: %s"
            % (request.goal_feature_name, request.tcp_name)
        )

        pure_goal_feature_name = ""
        for i in range(9):
            if request.goal_feature_name.endswith("_" + str(i)):
                pure_goal_feature_name = request.goal_feature_name[:-2]

        if pure_goal_feature_name != "":
            self.goal_feature_params_path = os.path.join(
                self.scene_parameters_path,
                "frames",
                "{name}.json".format(name=pure_goal_feature_name),
            )
        else:
            self.goal_feature_params_path = os.path.join(
                self.scene_parameters_path,
                "frames",
                "{name}.json".format(name=request.goal_feature_name),
            )
        if self.goal_feature_params_path != None:
            try:
                with open(self.goal_feature_params_path) as f:
                    self.goal_feature_params = json.load(f)
                    if "preferred_joint_configuration" in self.goal_feature_params:
                        self.have_preferred_joint_configuration = True
                        j0 = self.goal_feature_params["preferred_joint_configuration"][
                            "j0"
                        ]
                        j1 = self.goal_feature_params["preferred_joint_configuration"][
                            "j1"
                        ]
                        j2 = self.goal_feature_params["preferred_joint_configuration"][
                            "j2"
                        ]
                        j3 = self.goal_feature_params["preferred_joint_configuration"][
                            "j3"
                        ]
                        j4 = self.goal_feature_params["preferred_joint_configuration"][
                            "j4"
                        ]
                        j5 = self.goal_feature_params["preferred_joint_configuration"][
                            "j5"
                        ]
                    else:
                        self.have_preferred_joint_configuration = False
            except Exception:
                self.have_preferred_joint_configuration = False
                self.get_logger().info("no json item")
            # self.goal_feature_params = json.load(open(self.goal_feature_params_path))

        else:
            self.have_preferred_joint_configuration = False
            j0 = 0.0
            j1 = 0.0
            j2 = 0.0
            j3 = 0.0
            j4 = 0.0
            j5 = 0.0

        target_in_base = self.tf_lookup(BASEFRAME_ID, request.goal_feature_name, 3000)

        if not target_in_base.success:
            self.get_logger().error("Falied to lookup target in base.")

        tcp_in_faceplate = self.tf_lookup(FACEPLATE_ID, request.tcp_name)
        if not tcp_in_faceplate.success:
            self.get_logger().error("Falied to lookup tcp in faceplate.")

        ur_script = "def ur_script():\n"
        ur_script += "  set_tcp(" + self.pose_to_string(tcp_in_faceplate) + ")\n"
        if self.have_preferred_joint_configuration:
            ur_script += (
                "  movej(get_inverse_kin("
                + self.pose_to_string(target_in_base)
                + ", ["
                + str(j0)
                + " , "
                + str(j1)
                + ", "
                + str(j2)
                + ", "
                + str(j3)
                + ", "
                + str(j4)
                + ", "
                + str(j5)
                + "])"
                + ",a="
                + str(request.acceleration)
                + ",v="
                + str(request.velocity)
                + ")\n"
            )
        else:
            ur_script += (
                "movej(get_inverse_kin("
                + self.pose_to_string(target_in_base)
                + "),a="
                + str(request.acceleration)
                + ",v="
                + str(request.velocity)
                + ")\n"
            )
        ur_script += "end\nur_script()"

        if target_in_base.success and tcp_in_faceplate.success:
            return ur_script
        else:
            return ""

    def generate_move_l(self, request):
        target_in_base = self.tf_lookup(BASEFRAME_ID, request.goal_feature_name, 3000)
        if not target_in_base.success:
            self.get_logger().error("Falied to lookup target in base.")
        tcp_in_faceplate = self.tf_lookup(FACEPLATE_ID, request.tcp_name)
        if not tcp_in_faceplate.success:
            self.get_logger().error("Falied to lookup tcp in faceplate.")
        ur_script = "def ur_script():\n"
        ur_script += "  set_tcp(" + self.pose_to_string(tcp_in_faceplate) + ")\n"
        ur_script += (
            "  movel("
            + self.pose_to_string(target_in_base)
            + ",a="
            + str(request.acceleration)
            + ",v="
            + str(request.velocity)
            + ")\n"
        )
        ur_script += "end\nur_script()"
        if target_in_base.success and tcp_in_faceplate.success:
            return ur_script
        else:
            return ""

    def generate_set_standard_digital_out(self, io, value):
        ur_script = "def ur_script():\n"
        ur_script += "  set_standard_digital_out(" + str(io) + "," + str(value) + ")\n"
        ur_script += "end\nur_script()"
        return ur_script

    def freedrive_mode(self):
        ur_script = "def ur_script():\n"
        ur_script += "  while True:\n"
        ur_script += "    freedrive_mode()\n"
        ur_script += "  end\n"
        ur_script += "end\nur_script()"
        return ur_script

    def end_freedrive_mode(self):
        ur_script = "def ur_script():\n"
        ur_script += "  end_freedrive_mode()\n"
        ur_script += "end\nur_script()"
        return ur_script


def main(args=None):
    rclpy.init(args=args)
    ursg = URScriptGenerator()
    rclpy.spin(ursg)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
