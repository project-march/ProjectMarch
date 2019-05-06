import math


from JointSettingSpinBoxDelegate import JointSettingSpinBoxDelegate
from python_qt_binding.QtWidgets import QTableWidgetItem

from model.Setpoint import Setpoint
import subprocess

TABLE_DIGITS = 4


def notify(title, message):
    subprocess.Popen(['notify-send', str(title), str(message)])


def table_to_setpoints(table_data):
    setpoints = []
    for i in range(0, table_data.columnCount()):
        time = float(table_data.item(0, i).text())
        position = math.radians(float(table_data.item(1, i).text()))
        velocity = math.radians(float(table_data.item(2, i).text()))
        setpoints.append(Setpoint(time, position, velocity))
    return setpoints


def update_table(table, joint, duration):
    table.setColumnCount(len(joint.setpoints))

    for i in range(0, len(joint.setpoints)):

        time_item = QTableWidgetItem(str(round(joint.setpoints[i].time, TABLE_DIGITS)))

        position_item = QTableWidgetItem(
            str(round(math.degrees(joint.setpoints[i].position), TABLE_DIGITS)))

        velocity_item = QTableWidgetItem(
            str(round(math.degrees(joint.setpoints[i].velocity), TABLE_DIGITS)))

        table.setItem(0, i, time_item)
        table.setItem(1, i, position_item)
        table.setItem(2, i, velocity_item)

    table.setItemDelegate(JointSettingSpinBoxDelegate(joint.limits.velocity, joint.limits.lower, joint.limits.upper, duration))
    table.resizeRowsToContents()
    table.resizeColumnsToContents()
    return table

def plot_to_setpoints(plot):
    plot_data = plot.plot_item.getData()
    setpoints = []
    for i in range(0, len(plot_data[0])):
        velocity = plot.velocities[i]
        time = plot_data[0][i]
        position = math.radians(plot_data[1][i])
        setpoints.append(Setpoint(time, position, velocity))
    return setpoints


def update_ui_elements(joint, duration, table=None, plot=None):
    if plot is not None:
        plot.plot_item.blockSignals(True)
    if table is not None:
        table.blockSignals(True)

    if plot is not None:
        plot.updateSetpoints(joint)
    if table is not None:
        update_table(table, joint, duration)

    if plot is not None:
        plot.plot_item.blockSignals(False)
    if table is not None:
        table.blockSignals(False)
