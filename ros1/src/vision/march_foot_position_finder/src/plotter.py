import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QThread
from pubsub import pub


class Gui(QWidget):
    def __init__(self):
        super(Gui, self).__init__()

        self.text = QLabel(self)

        img = np.ones(shape=(80, 80, 3), dtype=np.uint8) * 255
        q_img = QPixmap(QImage(img.data, img.shape[0], img.shape[1], QImage.Format_RGB888)).scaled(800, 800)
        self.text.setPixmap(q_img)

        self.setGeometry(300, 300, 800, 800)
        self.show()

    def update_data(self, img):
        matrix = []

        img_filtered = img[img != -10]
        max_value = img_filtered.max()
        min_value = img_filtered.min()

        for i in range(img.shape[0]):
            row = []
            for j in range(img.shape[1]):
                if max_value != min_value and img[i, j] != -10:
                    value = (img[i, j] - min_value) * (1 / (max_value - min_value) * 150)
                else:
                    value = 255
                pixel = np.array([value, value, value], dtype=np.uint8)
                row.append(pixel)
            matrix.append(row)

        img = np.array(matrix)
        q_img = QPixmap(QImage(img.data, img.shape[0], img.shape[1], QImage.Format_RGB888)).scaled(800, 800)
        self.text.setPixmap(q_img)


# ROS - Worker Thread
class RosThread(QThread):
    def __init__(self, topic, parent=None):
        QThread.__init__(self, parent)
        self.sub = rospy.Subscriber(topic, Float64MultiArray, self.callback)
        self.n = 0

    def run(self):
        rospy.spin()

    def callback(self, msg: Float64MultiArray):
        img_list = np.array(msg.data)
        if msg.layout.data_offset:
            side = int(np.sqrt(len(img_list)))
            img = img_list.reshape(side, side)
            pub.sendMessage("update", img=img)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = Gui()

    # connect method 'gui.setLabel' with topic 'update'
    pub.subscribe(gui.update_data, "update")

    rospy.init_node("listener")
    ros1 = RosThread("/debug/height_map")
    ros1.start()

    sys.exit(app.exec_())
