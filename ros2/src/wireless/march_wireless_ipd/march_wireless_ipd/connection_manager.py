from numpy import isin
import select
from march_rqt_input_device.input_device_controller import InputDeviceController
from march_shared_msgs.msg import CurrentGait

import socket
import json
import traceback
import time
from functools import partial
from datetime import datetime


class ConnectionManager:
    def __init__(self, host, port, controller, node):
        server_address = (host, port)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind(server_address)
        self.s.listen()
        self.controller = controller
        self.seq = 0
        self.stop_receive = False
        self.requested_gait = None
        self.current_gait = "unknown"
        self.last_heartbeat = datetime.now()
        self.node = node
        self.stopped = False
        self.controller.accepted_cb = partial(
            self.send_message_till_confirm, "Accepted", True
        )
        self.controller.rejected_cb = partial(self.send_message_till_confirm, "Reject")
        self.controller.current_gait_cb = self._current_gait_cb
        self.controller.finished_cb = self.gait_finished

    def _current_gait_cb(self, msg):

        if self.stopped or "close" in msg.subgait:
            self.current_gait = self.requested_gait = "stop"
            self.stopped = False
        else:
            self.current_gait = msg.gait

    def wait_for_request(self):
        while True:
            try:
                self.empty_socket()

                counter = 0
                while self.stop_receive and counter < 15:
                    time.sleep(0.20)
                    counter += 1

                req = self.wait_for_message(1.0)

                if req == "":
                    self.controller.get_node().get_logger().warning(
                        "Connection lost with wireless IPD"
                    )
                    raise Exception("Connection lost")
                    # self.reset_connection()
                else:
                    self.last_heartbeat = datetime.now()

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
                        self.current_gait = self.requested_gait = "stop"
                        self.send_message_till_confirm("Accepted", True)
                    else:
                        self.request_gait(req)
                elif "Heartbeat" in req:
                    req = json.loads(req)
                    self.seq = req["seq"]

                    self.send_message_till_confirm("Heartbeat")
                    # Wait untill response comes back from android IPD

            except socket.timeout:
                if (datetime.now() - self.last_heartbeat).total_seconds() > 5.0:
                    self.controller.get_node().get_logger().warning(
                        "Connection lost with wireless IPD (no heartbeat)"
                    )
                    raise Exception("Connection lost")

            except Exception as e:
                print(e)
                raise e

    def gait_finished(self):
        pass

    def request_gait(self, req):
        self.stop_receive = True

        self.controller.update_possible_gaits()
        future = self.controller.get_possible_gaits()

        counter = 0
        while not future.done() and counter < 50:
            time.sleep(0.010)
            counter += 1

        self.requested_gait = req["gait"]["gaitName"]

        if self.requested_gait in future.result().gaits:
            self.controller.get_node().get_logger().info("Succesful gait")
            self.controller.publish_gait(self.requested_gait)
            return True
        else:
            self.controller.get_node().get_logger().info("Failed gait")
            self.send_message_till_confirm("Fail")
            return False

    def wait_for_message(self, timeout=5.0):
        try:
            self.connection.settimeout(timeout)
            data = self.connection.recv(1024).decode("utf-8")
            self.connection.settimeout(None)
        except Exception:
            raise
        return data

    def send_message(self, msg):
        try:
            msg = msg + "\r\n"
            self.connection.sendall(msg.encode())
        except BrokenPipeError:
            raise
        except Exception:
            self.controller.get_node().get_logger().warning(traceback.format_exc())

    def send_message_till_confirm(self, type, requested_gait=False):

        if requested_gait:
            sendGait = self.requested_gait
        else:
            sendGait = self.current_gait

        if self.connection is None:
            return

        msg = {"type": type, "currentGait": sendGait, "seq": self.seq}

        while True:
            try:
                self.send_message(json.dumps(msg))
                data = self.wait_for_message(0.40)

                if data == "":
                    self.controller.get_node().get_logger().warning(
                        "Connection lost with wireless IPD"
                    )
                    self.empty_socket()
                    raise Exception("Connection lost")
                else:
                    self.last_heartbeat = datetime.now()

                if "Received" in data:
                    data = json.loads(data)
                    if data["seq"] == self.seq:
                        self.stop_receive = False
                        self.empty_socket()
                        return
                    else:
                        self.controller.get_node().get_logger().warning("Different seq")

            except socket.timeout:
                self.controller.get_node().get_logger().info("Socket timeout")
                if (datetime.now() - self.last_heartbeat).total_seconds() > 5.0:
                    self.controller.get_node().get_logger().warning(
                        "Connection lost with wireless IPD (no heartbeat 2)"
                    )
                    self.empty_socket()
                    raise Exception("Connection lost")

            except Exception as e:
                self.empty_socket()
                raise e

    def establish_connection(self):
        while True:
            try:
                self.connection, self.addr = self.s.accept()
                self.controller.get_node().get_logger().info("Wireless IPD connected")
                self.wait_for_request()

            except Exception as e:
                print(e, "(no traceback)")

            self.connection.close()

    def empty_socket(self):
        input = [self.s]
        while 1:
            input_ready, _, _ = select.select(input, [], [], 0.0)
            if len(input_ready) == 0:
                break
            for s in input_ready:
                s.recv(1)
