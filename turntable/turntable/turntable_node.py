import time

import rclpy
import turntable.ortery_driver as driver
from turntable.ortery_driver import SSHOptions
from rclpy.action import ActionServer
from rclpy.node import Node
from turntable_interfaces.msg import CommandDesc, \
                                     PropertyDesc
from turntable_interfaces.srv import GetCommandDesc, \
                                     GetDeviceCount, \
                                     GetDeviceInfo, \
                                     GetPropertyDesc, \
                                     GetPropertyData, \
                                     SendCommand, \
                                     SetPropertyData, \
                                     SetPropertiesData, \
                                     TurntableStop
from turntable_interfaces.action import Turntable, \
                                        TurntableDegrees

def map_ortery_command_desc_to_ros_type(ocd):
    desc = CommandDesc()
    desc.name = ocd.name
    desc.value = ocd.value
    desc.description = ocd.description
    return desc


def map_ortery_property_desc_to_ros_type(opd):
    desc = PropertyDesc()
    desc.name = opd.name
    desc.value = opd.value
    desc.description = opd.description
    return desc


class TurntableNode(Node):
    def __init__(self):
        super().__init__("turntable_node")
        self.get_device_count = self.create_service(
            GetDeviceCount,
            "get_device_count",
            self.get_device_count_callback)
        self.get_device_info =  self.create_service(
            GetDeviceInfo,
            "get_device_info",
            self.get_device_info_callback)
        self.get_command_desc =  self.create_service(
            GetCommandDesc,
            "get_command_desc",
            self.get_command_desc_callback)
        self.get_property_desc = self.create_service(
            GetPropertyDesc,
            "get_property_desc",
            self.get_property_desc_callback)
        self.get_property_data = self.create_service(
            GetPropertyData,
            "get_property_data",
            self.get_property_data_callback)
        self.set_property_data = self.create_service(
            SetPropertyData,
            "set_property_data",
            self.set_property_data_callback)
        self.set_properties_data = self.create_service(
            SetPropertiesData,
            "set_properties_data",
            self.set_properties_data_callback)
        self.send_command = self.create_service(
            SendCommand,
            "send_command",
            self.send_command_callback)
        self.turntable = ActionServer(
            self,
            Turntable,
            "turntable",
            self.turntable_callback)
        self.turntable_degrees = ActionServer(
            self,
            TurntableDegrees,
            "turntable_degrees",
            self.turntable_degrees_callback)
        self.turntable_stop = self.create_service(
            TurntableStop,
            "turntable_stop",
            self.turntable_stop_callback)
        self.declare_parameter("debug", False)
        self.declare_parameter("use_ssh", False)
        self.declare_parameter("ssh_user", "")
        self.declare_parameter("ssh_host", "")
        self.declare_parameter("ssh_password", "")

    def get_debug_value(self):
        return self.get_parameter("debug").get_parameter_value().bool_value

    def get_ssh_options(self):
        if self.get_parameter("use_ssh").get_parameter_value().bool_value:
            user = str(self.get_parameter("ssh_user").value)
            host = str(self.get_parameter("ssh_host").value)
            password = str(self.get_parameter("ssh_password").value)
            return SSHOptions(user, host, password if password != "" else None)
        return None
        
    def get_device_count_callback(self, request, response):
        response.count = driver.get_device_count(self.get_debug_value(), self.get_ssh_options())
        return response

    def get_device_info_callback(self, request, response):
        try:
            device_info = driver.get_device_info(request.id,
                                                 self.get_debug_value(),
                                                 self.get_ssh_options())
            response.product_name = device_info.product_name
            response.device_i = device_info.device_i
            response.success = True
        except InvalidIdException:
            response.success = False

        return response

    def get_command_desc_callback(self, request, response):
        try:
            command_descs = driver.get_command_desc(request.device_i,
                                                    self.get_debug_value(),
                                                    self.get_ssh_options())
            response.command_descs = [
                map_ortery_command_desc_to_ros_type(command_desc)
                for command_desc in command_descs]
            response.success = True
        except InvalidIdException:
            response.success = False
        return response

    def get_property_desc_callback(self, request, response):
        try:
            property_descs = driver.get_property_desc(request.device_i,
                                                      self.get_debug_value(),
                                                      self.get_ssh_options())
            response.property_descs = [
                map_ortery_property_desc_to_ros_type(property_desc)
                for property_desc in property_descs]
            response.success = True
        except InvalidIdException:
            response.success = False
        return response

    def get_property_data_callback(self, request, response):
        try:
            response.data = driver.get_property_data(request.device_i,
                                                     request.property_id,
                                                     self.get_debug_value(),
                                                     self.get_ssh_options())
            response.success = True
        except InvalidIdException:
            response.success = False
        return response

    def set_property_data_callback(self, request, response):
        try:
            result.success = driver.set_property_data(request.device_i,
                                                      request.property_id,
                                                      request.data,
                                                      self.get_debug_value(),
                                                      self.get_ssh_options())
        except:
            response.success = False
        return response

    def set_properties_data_callback(self, request, response):
        try:
            response.success = driver.set_properties_data(request.device_i,
                                                          request.properties,
                                                          request.data,
                                                          self.get_debug_value(),
                                                          self.get_ssh_options())
        except:
            response.success = False
        return response

    def send_command_callback(self, request, response):
        try:
            response.success = driver.send_command(request.device_i,
                                                   request.command,
                                                   self.get_debug_value(),
                                                   self.get_ssh_options())
        except:
            response.success = False
        return response

    def turntable_callback(self, goal_handle):
        try:
            driver.turntable(goal_handle.request.device_i,
                             goal_handle.request.speed,
                             goal_handle.request.direction,
                             goal_handle.request.step,
                             self.get_debug_value(),
                             self.get_ssh_options())
            while driver.get_property_data(
                    goal_handle.request.device_i,
                    16641,
                    self.get_debug_value(),
                    self.get_ssh_options()) == 8210:
                feedback_msg = Turntable.Feedback()
                feedback_msg.state = "Turning"
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(1)
            success = True
            feedback_msg = Turntable.Feedback()
            feedback_msg.state = "Finished"
            goal_handle.publish_feedback(feedback_msg)
        except:
            success = False
        goal_handle.succeed()
        result = Turntable.Result()
        result.success = success
        return response

    def turntable_degrees_callback(self, goal_handle):
        try:
            total_steps = driver.get_property_data(goal_handle.request.device_i,
                                                   16643,
                                                   self.get_debug_value(),
                                                   self.get_ssh_options())
            driver.turntable(goal_handle.request.device_i,
                             goal_handle.request.speed,
                             goal_handle.request.direction,
                             int(goal_handle.request.degrees * (total_steps/360)),
                             self.get_debug_value(),
                             self.get_ssh_options())
            while driver.get_property_data(
                    goal_handle.request.device_i,
                    16641,
                    self.get_debug_value(),
                    self.get_ssh_options()) == 8210:
                feedback_msg = TurntableDegrees.Feedback()
                feedback_msg.state = "Turning"
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(1)
            success = True
            feedback_msg = TurntableDegrees.Feedback()
            feedback_msg.state = "Finished"
            goal_handle.publish_feedback(feedback_msg)
        except e:
            success = False
        goal_handle.succeed()
        result = TurntableDegrees.Result()
        result.success = success
        return result

    def turntable_stop_callback(self, request, response):
        try:
            response.success = driver.send_command(request.device_i,
                                                   13057,
                                                   self.get_debug_value(),
                                                   self.get_ssh_options())
        except:
            response.success = False
        return response


def main(args=None):
    print("Starting turntable node")
    rclpy.init(args=args)

    turntable_node = TurntableNode()

    rclpy.spin(turntable_node)

    turntable_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
