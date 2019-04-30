import math

import rospy

from python_qt_binding.QtWidgets import QTableWidgetItem, QMessageBox, QDoubleSpinBox, QAbstractSpinBox
from pyqtgraph.Qt import QtCore

import pynotify

from model.Setpoint import Setpoint

TABLE_DIGITS = 4


def notify(title, message):
    pynotify.init("Basic")
    n = pynotify.Notification(title, message)
    n.set_urgency(pynotify.URGENCY_CRITICAL)
    n.show()


def table_to_setpoints(table_data):
    setpoints = []
    for i in range(0, table_data.columnCount()):
        time = table_data.cellWidget(0, i).value()
        position = math.radians(table_data.cellWidget(1, i).value())
        velocity = math.radians(table_data.cellWidget(2, i).value())
        setpoints.append(Setpoint(time, position, velocity))
    return setpoints


def update_table(table, joint, duration):
    table.setColumnCount(len(joint.setpoints))

    for i in range(0, len(joint.setpoints)):

        time_offset = 1/math.pow(10, TABLE_DIGITS)

        if i == 0:
            min_time = 0
            max_time = joint.setpoints[i+1].time - time_offset
        elif i == len(joint.setpoints)-1:
            min_time = joint.setpoints[i-1].time + time_offset
            max_time = duration
        else:
            min_time = joint.setpoints[i-1].time + time_offset
            max_time = joint.setpoints[i+1].time - time_offset

        table.setCellWidget(0, i, create_table_spinbox(table.item(0, i), joint.setpoints[i].time, min_time, max_time))
        table.setCellWidget(1, i, create_table_spinbox(table.item(1, i), math.degrees(joint.setpoints[i].position), math.degrees(joint.limits.lower), math.degrees(joint.limits.upper)))
        table.setCellWidget(2, i, create_table_spinbox(table.item(2, i), math.degrees(joint.setpoints[i].velocity), math.degrees(-joint.limits.velocity), math.degrees(joint.limits.velocity)))

    table.resizeRowsToContents()
    table.resizeColumnsToContents()
    return table


def create_table_spinbox(spinbox, value, min, max):
    if spinbox is None:
        spinbox = QDoubleSpinBox()
    spinbox.setValue(value)
    spinbox.setDecimals(TABLE_DIGITS)
    spinbox.setMinimum(min)
    spinbox.setMaximum(max)
    spinbox.setButtonSymbols(QAbstractSpinBox.NoButtons)
    spinbox.setCorrectionMode(QAbstractSpinBox.CorrectToNearestValue)
    spinbox.setFixedWidth(75)
    spinbox.setSingleStep(0)
    return spinbox


def plot_to_setpoints(plot):
    plot_data = plot.plot_item.getData()
    setpoints = []
    for i in range(0, len(plot_data[0])):
        velocity = plot.velocities[i]
        time = plot_data[0][i]
        position = math.radians(plot_data[1][i])
        setpoints.append(Setpoint(time, position, velocity))
    return setpoints


def update_ui_elements(joint, table, plot, duration):
    plot.plot_item.blockSignals(True)
    table.blockSignals(True)

    plot.updateSetpoints(joint)
    update_table(table, joint, duration)

    plot.plot_item.blockSignals(False)
    table.blockSignals(False)
