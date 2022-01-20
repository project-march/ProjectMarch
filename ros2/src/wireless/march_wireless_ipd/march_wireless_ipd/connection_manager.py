from numpy import isin
from march_rqt_input_device.input_device_controller import InputDeviceController

import socket
import json
import traceback
import time
from functools import partial
from datetime import datetime



class ConnectionManager():
    
    def __init__(self, host, port, controller):
        server_address = (host, port)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind(server_address)
        self.s.listen()
        self.controller = controller
        self.controller.accepted_cb = partial(self.send_message_till_confirm, "accept")
        self.controller.rejected_cb = partial(self.send_message_till_confirm, "reject")
        self.controller.current_gait_cb = self.update_current_gait
        self.controller.finished_cb = self.gait_finished
        self.seq = -1
        self.stop_receive = False
        self.current_gait = "unknown"
        self.last_heartbeat = datetime.now()

    def wait_for_request(self):
        while True:
            try: 
                counter = 0
                while(self.stop_receive and counter < 15):
                    time.sleep(0.20)
                    counter += 1

                req = self.wait_for_message(1.0)

                if req == "":
                    self.controller.get_node().get_logger().warning("Connection lost with wireless IPD")
                    self.reset_connection()
                if "Received" in req:
                    continue
                elif "GaitRequest" in req:
                    req = json.loads(req)
                    self.seq = max(self.seq, req["seq"])
                
                    if req["gait"]["gaitName"] == "stop":
                        self.stop_receive = True
                        self.controller.update_possible_gaits()
                        self.controller.publish_stop()
                        self.send_message_till_confirm("accept")
                        self.current_gait = "stop"
                    else:
                        self.request_gait(req)
                else:
                    self.last_heartbeat = datetime.now()

            except socket.timeout:
                if (datetime.now() - self.last_heartbeat).total_seconds() > 1.5:
                    self.controller.get_node().get_logger().warning("Connection lost with wireless IPD (no heartbeat)")
                    self.reset_connection()

            except Exception:
                self.reset_connection()

    def update_current_gait(self, gait):        
        self.current_gait = gait

    def gait_finished(self):
        pass

    def request_gait(self, req):
        self.controller.update_possible_gaits()
        future = self.controller.get_possible_gaits()

        while(not future.done()):
            time.sleep(0.010)

        gait = req["gait"]

        if gait["gaitName"] in future.result().gaits:
            self.stop_receive = True
            self.controller.publish_gait(gait["gaitName"])
            return True
        else:
            self.stop_receive = True
            self.send_message_till_confirm("fail")
            return False


    def wait_for_message(self, timeout=5.0):
        try:
            self.connection.settimeout(timeout)
            data = self.connection.recv(1024).decode('utf-8')
            self.connection.settimeout(None)
        except Exception:
            raise
        return data


    def send_message(self, msg):
        try:
            msg = msg + '\r\n'
            self.connection.sendall(msg.encode())
        except BrokenPipeError:
            raise
        except Exception:
            self.controller.get_node().get_logger().warning(traceback.format_exc())


    def send_message_till_confirm(self, msg):
        if self.connection is None:
            return

        msg = {
            "msg": msg,
            "seq": self.seq
        }

        while True:
            try:
                self.send_message(json.dumps(msg))
                data = self.wait_for_message(0.20)

                if "Received" in data:
                    data = json.loads(data)
                    if data["seq"] == self.seq:
                        self.stop_receive = False
                        return
                else:
                    self.last_heartbeat = datetime.now()

            except socket.timeout:
                if (datetime.now() - self.last_heartbeat).total_seconds() > 1.5:
                    self.controller.get_node().get_logger().warning("Connection lost with wireless IPD (no heartbeat)")
                    self.reset_connection()

            except Exception:
                self.reset_connection()


    def establish_connection(self):
        self.connection, self.addr = self.s.accept()
        self.send_message_till_confirm(self.current_gait)
        self.controller.get_node().get_logger().info("Wireless IPD connected")
        self.wait_for_request()


    def reset_connection(self):
        self.connection.close()
        self.establish_connection()
