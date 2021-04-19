from python_qt_binding.QtWidgets import QCheckBox, QPushButton
import rospy


class SideSubgaitView(object):
    def __init__(self, widget, side: str = None):
        if side == "previous":
            self.import_button = widget.findChildren(
                QPushButton, "import_previous_subgait_button"
            )[0]
            self.default_checkbox = widget.findChildren(
                QCheckBox, "previous_is_standing_check_box"
            )[0]
            self.lock_checkbox = widget.findChildren(
                QCheckBox, "lock_startpoint_check_box"
            )[0]
        elif side == "next":
            self.import_button = widget.findChildren(
                QPushButton, "import_next_subgait_button"
            )[0]
            self.default_checkbox = widget.findChildren(
                QCheckBox, "next_is_standing_check_box"
            )[0]
            self.lock_checkbox = widget.findChildren(
                QCheckBox, "lock_endpoint_check_box"
            )[0]
        else:
            rospy.loginfo(
                "SideSubgaitView initialized without specified side, "
                "can raise issues with locking setpoints."
            )
            self.import_button = widget.findChildren(QPushButton)[0]
            self.default_checkbox = widget.findChildren(QCheckBox)[1]
            self.lock_checkbox = widget.findChildren(QCheckBox)[0]

    def update_widget(self, controller):
        self.import_button.setText(controller.subgait_text)

        self.lock_checkbox.blockSignals(True)
        self.lock_checkbox.setChecked(controller.lock_checked)
        self.lock_checkbox.blockSignals(False)

        self.default_checkbox.blockSignals(True)
        self.default_checkbox.setChecked(controller.default_checked)
        self.default_checkbox.blockSignals(False)
