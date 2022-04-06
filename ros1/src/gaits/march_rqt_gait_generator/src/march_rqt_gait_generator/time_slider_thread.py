from pyqtgraph.Qt import QtCore
import rospy


class TimeSliderThread(QtCore.QThread):

    update_signal = QtCore.pyqtSignal(int)

    def __init__(self, current, playback_speed, max_time):
        QtCore.QThread.__init__(self)
        self.current = current
        self.playback_speed = playback_speed
        self.max = max_time
        self.allowed_to_run = True

    def run(self):
        index = 0
        calculations_per_second = 30
        r = rospy.Rate(calculations_per_second)
        while self.allowed_to_run:
            index += 1
            value = (self.current + float(index) / calculations_per_second * self.playback_speed) % self.max
            self.update_signal.emit(value)
            r.sleep()

    def stop(self):
        self.allowed_to_run = False
