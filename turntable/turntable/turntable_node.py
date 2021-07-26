import re
import subprocess
import rclpy
from rclpy.node import Node
from turntable_srvs.srv import GetDeviceCount


def rwo(command):
    return subprocess.run(command, capture_output=True)

class TurntableNode(Node):
    def __init__(self):
        self.get_device_count = self.create_service(
                                    GetDeviceCount,
                                    'get_device_count',
                                    self.get_device_count_callback)
        self.get_device_info =  self.create_service(
                                    GetDeviceInfo,
                                    'get_device_info',
                                    self.get_device_info_callback)
        
    def get_device_count_callback(self, request, response):
        proc = rwo("OTADCommand.exe get_device_count")
        m = re.search('^([0-9]+)\\r\\n$', proc.stdout.decode("utf-8"))
        response.count = m.group(1)
        return response

    def get_device_info(self, request, response):
        proc = rwo(f"OTADCommand.exe get_device_info {request.id}")
        e = 'get_device_info :  command exec fail ( error code : 0x0040001)\r\n'
        if proc.stdout.decode('utf-8') == e:
            response.success = False
            return response

        s = '^Product Name : ([A-Za-z0-9 ]+)\\r\\nDevice ID : ([0-9]+)\\r\\n'
        m = re.search(s, proc.stdout.decode('utf-8'))
        response.product_name = m.group(1)
        response.device_id = m.group(2)
        response.success = True
        return response

                
                

def main(args=None):
    rclpy.init(args=args)

    turntable_node = TurntableNode()

    rclpy.spin(turntable_node)

    turntable_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
