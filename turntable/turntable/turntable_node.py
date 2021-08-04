import rclpy
import turntable.ortery_driver as driver
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
                                     Turntable, \
                                     TurntableDegrees, \
                                     TurntableStop


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
        self.turntable = self.create_service(
            Turntable,
            "turntable",
            self.turntable_callback)
        self.turntable_degrees = self.create_service(
            TurntableDegrees,
            "turntable_degrees",
            self.turntable_degrees_callback)
        self.turntable_stop = self.create_service(
            TurntableStop,
            "turntable_stop",
            self.turntable_stop_callback)
        self.declare_parameter("debug", False)

    def get_debug_value(self):
        return self.get_parameter("debug").get_parameter_value().bool_value
        
    def get_device_count_callback(self, request, response):
        response.count = driver.get_device_count(self.get_debug_value())
        return response

    def get_device_info_callback(self, request, response):
        try:
            device_info = driver.get_device_info(request.id,
                                                 self.get_debug_value())
            response.product_name = device_info.product_name
            response.device_i = device_info.device_i
            response.success = True
        except InvalidIdException:
            response.success = False

        return response

    def get_command_desc_callback(self, request, response):
        try:
            command_descs = driver.get_command_desc(request.device_i,
                                                    self.get_debug_value())
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
                                                      self.get_debug_value())
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
                                                     self.get_debug_value())
            response.success = True
        except InvalidIdException:
            response.success = False
        return response

    def set_property_data_callback(self, request, response):
        try:
            result.success = driver.set_property_data(request.device_i,
                                                      request.property_id,
                                                      request.data,
                                                      self.get_debug_value())
        except:
            response.success = False
        return response

    def set_properties_data_callback(self, request, response):
        try:
            response.success = driver.set_properties_data(request.device_i,
                                                          request.properties,
                                                          request.data,
                                                          self.get_debug_value())
        except:
            response.success = False
        return response

    def send_command_callback(self, request, response):
        try:
            response.success = driver.send_command(request.device_i,
                                                   request.command,
                                                   self.get_debug_value())
        except:
            response.success = False
        return response

    def turntable_callback(self, request, response):
        try:
            response.success = driver.turntable(request.device_i,
                                                request.speed,
                                                request.direction,
                                                request.step,
                                                self.get_debug_value())
        except:
            response.success = False
        return response

    def turntable_degrees_callback(self, request, response):
        try:
            total_steps = driver.get_property_data(request.device_i,
                                                   16643,
                                                   self.get_debug_value())
            response.success = driver.turntable(request.device_i,
                                                request.speed,
                                                request.direction,
                                                int(request.degrees * (total_steps/360)),
                                                self.get_debug_value())
        except:
            response.success = False
        return response

    def turntable_stop_callback(self, request, response):
        try:
            response.success = driver.send_command(request.device_i,
                                                   13057,
                                                   self.get_debug_value())
        except:
            response.success = False
        return response


def main(args=None):
    rclpy.init(args=args)

    turntable_node = TurntableNode()

    rclpy.spin(turntable_node)

    turntable_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
