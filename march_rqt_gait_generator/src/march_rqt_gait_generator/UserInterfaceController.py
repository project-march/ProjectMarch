import math

import rospy

from python_qt_binding.QtWidgets import QTableWidgetItem, QMessageBox, QDoubleSpinBox, QAbstractSpinBox
from pyqtgraph.Qt import QtCore

from model.Setpoint import Setpoint

TABLE_DIGITS = 4


def table_to_setpoints(table_data):
    setpoints = []
    for i in range(0, table_data.columnCount()):
        time = float(table_data.item(0, i).value())
        position = math.radians(float(table_data.item(1, i).value()))
        velocity = math.radians(float(table_data.item(2, i).value()))
        setpoints.append(Setpoint(time, position, velocity))
    return setpoints


def update_table(table, setpoints, duration):
    rospy.logdebug("Updating table")

    table.setColumnCount(len(setpoints))
    for i in range(0, len(setpoints)):

        if i == 0:
            min_time = 0
            max_time = setpoints[i+1].time - 0.001
        elif i == len(setpoints)-1:
            min_time = setpoints[i-1].time + 0.001
            max_time = duration
        else:
            min_time = setpoints[i-1].time + 0.001
            max_time = setpoints[i+1].time - 0.001

        print table
        table.setCellWidget(0, i, update_table_spinbox(table.cellWidget(0, i), setpoints[i].time, min_time, max_time))

        position_item = QTableWidgetItem(str(round(math.degrees(setpoints[i].position), TABLE_DIGITS)))
        table.setItem(1, i, position_item)
        velocity_item = QTableWidgetItem(str(round(math.degrees(setpoints[i].velocity), TABLE_DIGITS)))
        table.setItem(2, i, velocity_item)

    table.resizeRowsToContents()
    table.resizeColumnsToContents()
    return table


def update_table_spinbox(spinbox, value, min, max):
    if spinbox is None:
        spinbox = QDoubleSpinBox()
    spinbox.setValue(value)
    spinbox.setDecimals(TABLE_DIGITS)
    spinbox.setMinimum(min)
    spinbox.setMaximum(max)
    spinbox.setButtonSymbols(QAbstractSpinBox.NoButtons)
    spinbox.setCorrectionMode(QAbstractSpinBox.CorrectToNearestValue)
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

    update_table(table, joint.setpoints, duration)

    plot.plot_item.blockSignals(False)
    table.blockSignals(False)
