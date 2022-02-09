from numpy import isin
from march_rqt_input_device.input_device_controller import InputDeviceController
from march_shared_msgs.msg import CurrentGait

import socket
import json
import traceback
import time
from functools import partial
from datetime import datetime



class ConnectionManager():
    
    def __init__(self, host, port, controller, node):
        server_address = (host, port)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind(server_address)
        self.s.listen()
        self.controller = controller
        self.controller.accepted_cb = partial(self.send_message_till_confirm, "accept")
        self.controller.rejected_cb = partial(self.send_message_till_confirm, "reject")
        self.controller.current_gait_cb = self._current_gait_cb
        self.controller.finished_cb = self.gait_finished
        self.seq = 0
        self.stop_receive = False
        self.current_gait = "unknown"
        self.last_heartbeat = datetime.now()
        self.node = node
        self.stopped = False

    def _current_gait_cb(self, msg):
        if self.stopped:
            self.controller.get_node().get_logger().info("UPDATE stop")
            self.current_gait = "stop"
            self.stopped = False
        else:
            self.controller.get_node().get_logger().info("UPDATE " + msg)
            self.current_gait = msg

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
                    raise Exception("Connection lost")
                    # self.reset_connection()

                if "Received" in req:
                    continue
                elif "GaitRequest" in req:
                    req = json.loads(req)
                    self.seq = req["seq"]

                    if req["gait"]["gaitName"] == "stop":
                        self.stop_receive = True
                        self.controller.update_possible_gaits()
                        self.controller.publish_stop()
                        self.stopped = True
                        self.send_message_till_confirm("accept")
                    else:
                        self.request_gait(req)
                elif "Heartbeat" in req:
                    req = json.loads(req)
                    self.req = req["seq"]

                    self.send_message_till_confirm(self.current_gait)
                    # Wait untill response comes back from android IPD

                    
                else:
                    self.last_heartbeat = datetime.now()

            except socket.timeout:
                if (datetime.now() - self.last_heartbeat).total_seconds() > 1.5:
                    self.controller.get_node().get_logger().warning("Connection lost with wireless IPD (no heartbeat)")
                    raise Exception("Connection lost")

            except Exception as e:
                raise e

            # except Exception:
            #     traceback.print_exc()
            #     self.reset_connection()

    def gait_finished(self):
        pass

    def request_gait(self, req):
        self.controller.update_possible_gaits()
        future = self.controller.get_possible_gaits()

        counter = 0
        while(not future.done() and counter < 50):
            time.sleep(0.010)
            counter += 1

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

        self.controller.get_node().get_logger().warning(str(self.seq))    
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
                        self.controller.get_node().get_logger().warning("Different seq")    
                else:
                    self.last_heartbeat = datetime.now()
        
            except socket.timeout:
                if (datetime.now() - self.last_heartbeat).total_seconds() > 1.5:
                    self.controller.get_node().get_logger().warning("Connection lost with wireless IPD (no heartbeat 2)")
                    raise Exception("Connection lost")
            
            except Exception as e:
                raise e

            # except Exception:
            #     traceback.print_exc()
            #     self.reset_connection()


    def establish_connection(self):
        while True:
            try:
                self.connection, self.addr = self.s.accept()
                self.send_message_till_confirm(self.current_gait)
                self.controller.get_node().get_logger().info("Wireless IPD connected")
                self.wait_for_request()

            except Exception:
                traceback.print_exc()
                self.connection.close()


    # def reset_connection(self):
    #     self.connection.close()
    #     self.establish_connection()
