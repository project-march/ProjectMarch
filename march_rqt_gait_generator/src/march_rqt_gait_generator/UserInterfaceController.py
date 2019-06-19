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
    for i in range(0, table_data.rowCount()):
        time = float(table_data.item(i, 0).text())
        position = math.radians(float(table_data.item(i, 1).text()))
        velocity = math.radians(float(table_data.item(i, 2).text()))
        setpoints.append(Setpoint(time, position, velocity))
    return setpoints


def update_table(table, joint, duration):
    table.setRowCount(len(joint.setpoints))

    for i in range(0, len(joint.setpoints)):

        time_item = QTableWidgetItem(str(round(joint.setpoints[i].time, TABLE_DIGITS)))

        position_item = QTableWidgetItem(
            str(round(math.degrees(joint.setpoints[i].position), TABLE_DIGITS)))

        velocity_item = QTableWidgetItem(
            str(round(math.degrees(joint.setpoints[i].velocity), TABLE_DIGITS)))

        table.setItem(i, 0, time_item)
        table.setItem(i, 1, position_item)
        table.setItem(i, 2, velocity_item)

    table.setItemDelegate(JointSettingSpinBoxDelegate(
        joint.limits.velocity, joint.limits.lower, joint.limits.upper, duration))
    # table.resizeRowsToContents()
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


def update_ui_elements(joint, duration, table=None, plot=None, show_velocity_markers=False):
    if plot is not None:
        plot.plot_item.blockSignals(True)
    if table is not None:
        table.blockSignals(True)

    if plot is not None:
        plot.updateSetpoints(joint, show_velocity_markers)
    if table is not None:
        update_table(table, joint, duration)

    if plot is not None:
        plot.plot_item.blockSignals(False)
    if table is not None:
        table.blockSignals(False)
