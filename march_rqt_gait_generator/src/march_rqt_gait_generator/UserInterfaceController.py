import math

import rospy

from python_qt_binding.QtWidgets import QTableWidgetItem, QMessageBox


from model.Setpoint import Setpoint

TABLE_DIGITS = 4


def table_to_setpoints(table_data):
    setpoints = []
    for i in range(0, table_data.columnCount()):
        time = float(table_data.item(0, i).text())
        position = math.radians(float(table_data.item(1, i).text()))
        velocity = math.radians(float(table_data.item(2, i).text()))
        setpoints.append(Setpoint(time, position, velocity))
    return setpoints


def update_table(table, setpoints):
    rospy.logdebug("Updating table")

    table.setColumnCount(len(setpoints))
    for i in range(0, len(setpoints)):
        table.setItem(0, i, QTableWidgetItem(str(round(setpoints[i].time, TABLE_DIGITS))))
        table.setItem(1, i, QTableWidgetItem(str(round(math.degrees(setpoints[i].position), TABLE_DIGITS))))
        table.setItem(2, i, QTableWidgetItem(str(round(math.degrees(setpoints[i].velocity), TABLE_DIGITS))))

    table.resizeRowsToContents()
    table.resizeColumnsToContents()
    return table

def item_changed(self, Qitem):
    try:
        test = float(Qitem.text())
    except ValueError:
        Msgbox = QMessageBox()
        Msgbox.setText("Error, value must be number!")
        Msgbox.exec_()
        Qitem.setText(str(0.1))

def plot_to_setpoints(plot):
    plot_data = plot.plot_item.getData()
    setpoints = []
    for i in range(0, len(plot_data[0])):
        velocity = plot.velocities[i]
        time = plot_data[0][i]
        position = math.radians(plot_data[1][i])
        setpoints.append(Setpoint(time, position, velocity))
    return setpoints


def update_ui_elements(joint, table, plot):
    plot.plot_item.blockSignals(True)
    table.blockSignals(True)

    plot.updateSetpoints(joint)

    update_table(table, joint.setpoints)

    plot.plot_item.blockSignals(False)
    table.blockSignals(False)
