import math
import subprocess

from python_qt_binding.QtWidgets import QTableWidgetItem

from .joint_setting_spin_box_delegate import JointSettingSpinBoxDelegate
from .model.modifiable_setpoint import ModifiableSetpoint

TABLE_DIGITS = 4


def notify(title, message):
    subprocess.Popen(['notify-send', str(title), str(message)])


def table_to_setpoints(table_data):
    setpoints = []
    for i in range(0, table_data.rowCount()):
        time = float(table_data.item(i, 0).text())
        position = math.radians(float(table_data.item(i, 1).text()))
        velocity = math.radians(float(table_data.item(i, 2).text()))
        setpoints.append(ModifiableSetpoint(time, position, velocity))
    return setpoints


def update_table(table, joint):
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
        joint.limits.velocity, joint.limits.lower, joint.limits.upper, joint.duration))
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
        setpoints.append(ModifiableSetpoint(time, position, velocity))
    return setpoints
