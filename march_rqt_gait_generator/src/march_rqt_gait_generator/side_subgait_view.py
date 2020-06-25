from python_qt_binding.QtWidgets import (QCheckBox, QPushButton)


class SideSubgaitView(object):
    def __init__(self, widget):
        self.import_button = widget.findChildren(QPushButton)[0]
        self.default_checkbox = widget.findChildren(QCheckBox)[0]
        self.lock_checkbox = widget.findChildren(QCheckBox)[1]

    def update_widget(self, controller):
        self.import_button.setText(controller.subgait_text)

        self.lock_checkbox.blockSignals(True)
        self.lock_checkbox.setChecked(controller.lock_checked)
        self.lock_checkbox.blockSignals(False)

        self.default_checkbox.blockSignals(True)
        self.default_checkbox.setChecked(controller.default_checked)
        self.default_checkbox.blockSignals(False)
