import math

from python_qt_binding.QtWidgets import QTableWidgetItem

from .joint_setting_spin_box_delegate import JointSettingSpinBoxDelegate
from .model.modifiable_setpoint import ModifiableSetpoint


class JointTableController(object):
    TABLE_DIGITS = 4

    def __init__(self, joint_table_widget, joint):
        self.table_widget = joint_table_widget
        self.update_setpoints(joint)

    def update_setpoints(self, joint):
        self.table_widget.setRowCount(len(joint.setpoints))

        for i, setpoint in enumerate(joint.setpoints):
            time_item = QTableWidgetItem(str(round(setpoint.time, self.TABLE_DIGITS)))

            position_item = QTableWidgetItem(
                str(round(math.degrees(setpoint.position), self.TABLE_DIGITS)))

            velocity_item = QTableWidgetItem(
                str(round(math.degrees(setpoint.velocity), self.TABLE_DIGITS)))

            self.table_widget.setItem(i, 0, time_item)
            self.table_widget.setItem(i, 1, position_item)
            self.table_widget.setItem(i, 2, velocity_item)

        self.table_widget.setItemDelegate(JointSettingSpinBoxDelegate(
            joint.limits.velocity, joint.limits.lower, joint.limits.upper, joint.duration))
        # self.table_widget.resizeRowsToContents()
        self.table_widget.resizeColumnsToContents()

    def to_setpoints(self):
        setpoints = []
        for i in range(0, self.table_widget.rowCount()):
            time = float(self.table_widget.item(i, 0).text())
            position = math.radians(float(self.table_widget.item(i, 1).text()))
            velocity = math.radians(float(self.table_widget.item(i, 2).text()))
            setpoints.append(ModifiableSetpoint(time, position, velocity))
        return setpoints
