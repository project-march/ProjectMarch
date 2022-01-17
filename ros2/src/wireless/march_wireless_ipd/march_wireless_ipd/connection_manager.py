from march_rqt_input_device.input_device_controller import InputDeviceController

import socket
import json



class ConnectionManager():
    
    def __init__(self, host, port, controller):
        server_address = (host, port)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind(server_address)
        self.s.listen()
        self.controller = controller
        self.controller.accepted_cb = self.send_success
        self.controller.rejected_cb = self.send_fail
        self.controller.current_gait_cb = self.current_gait

    def wait_for_request(self):
        while True:
            if self.connection._closed:
                self.reset_connection()
            else:

                try:
                    req = self.wait_for_message(30)
                    if req == "":
                        break
                    
                    req = json.loads(req)

                    self.controller.get_node().get_logger().info(str(req))

                    self.seq = max(self.req, req["seq"])

                    if req["type"] == "GaitRequest":
                        self.controller.get_node().get_logger().info("Yes")
                        if req["gait"] == "stop":
                            self.controller.get_node().get_logger().info("stomp")
                            self.controller.publish_stop()
                        else:
                            self.controller.get_node().get_logger().info("request")
                            self.request_gait(req)
                    else:
                        self.controller.get_node().get_logger().info("No requesto")
                                

                except Exception:
                    break

        self.reset_connection()


    def current_gait(self, gait):
        
        self.controller.get_node().get_logger().info("Device connected")
        print(gait)
        pass


    def send_success(self):
        self.controller.get_node().get_logger().info("Sending success")
        msg = {
            "gait": "Success",
            "seq": self.seq,
            "type": "Exoskeleton"
        }
        self.send_message_till_confirm(json.dumps(msg))

    
    def send_fail(self):
        self.controller.get_node().get_logger().info("Sending fail")
        msg = {
            "gait": "Fail",
            "seq": self.seq,
            "type": "Exoskeleton"
        }
        self.send_message_till_confirm(json.dumps(msg))


    def request_gait(self, req):
        self.controller.get_node().get_logger().info("Requesting gait")
        self.controller.update_possible_gaits()
        future = self.controller.get_possible_gaits()
        print("Future", future)
        if req["gait"] in future.result():
            self.controller.publish_gait(req["gait"])
            return True
        else:
            self.send_fail(req["seq"])
            return False


    def wait_for_message(self, timeout=5.0):
        self.controller.get_node().get_logger().info("Waiting for message")
        try:
            self.controller.get_node().get_logger().info("A")
            self.connection.settimeout(timeout)
            self.controller.get_node().get_logger().info("B")
            data = self.connection.recv(1024).decode('utf-8')
            self.controller.get_node().get_logger().info("C")
            self.connection.settimeout(None)
            self.controller.get_node().get_logger().info("D")
        except Exception as e:
            self.controller.get_node().get_logger().warning(e)
            raise
        self.controller.get_node().get_logger().info("E")
        return data


    def send_message(self, msg):
        self.controller.get_node().get_logger().info("Sending single message")
        try:
            msg = msg + '\r\n'
            self.connection.sendall(msg.encode())
        except BrokenPipeError:
            raise


    def send_message_till_confirm(self, msg):
        self.controller.get_node().get_logger().info("Send till wait")
        while (True):
            while (True):
                try:
                    self.send_message(msg)
                    data = self.wait_for_message(0.20)
                    return data
                except socket.timeout:
                    continue
                except Exception:
                    break
            self.reset_connection()


    def establish_connection(self):
        self.connection, self.addr = self.s.accept()
        self.controller.get_node().get_logger().info("Device connected")
        self.wait_for_request()


    def reset_connection(self):
        self.connection.close()
        self.establish_connection()
