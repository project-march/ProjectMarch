import math

from pyqtgraph.Qt import QtCore, QtGui
from python_qt_binding.QtWidgets import QAbstractSpinBox, QDoubleSpinBox


class JointSettingSpinBoxDelegate(QtGui.QItemDelegate):
    """Delegate for the QDoubleSpinBox that opens when you edit a table cell.

    The min and max allowed value is based on the row (time, position, velocity).
    """

    def __init__(self, velocity_limit, min_position, max_position, duration, digits=4):
        super(JointSettingSpinBoxDelegate, self).__init__()
        self.velocity_limit = velocity_limit
        self.min_position = min_position
        self.max_position = max_position
        self.duration = duration
        self.digits = digits

    def createEditor(self, parent, option, index):

        time_offset = 1 / math.pow(10, self.digits)

        row = index.row()
        column = index.column()

        editor = QDoubleSpinBox(parent)

        # Time
        if column == 0:

            min_time_cell = index.model().data(index.sibling(row - 1, column), QtCore.Qt.EditRole)
            if min_time_cell is None:
                min_time = 0
            else:
                min_time = float(unicode(min_time_cell)) + time_offset

            max_time_cell = index.model().data(index.sibling(row + 1, column), QtCore.Qt.EditRole)
            if max_time_cell is None:
                max_time = self.duration
            else:
                max_time = float(unicode(max_time_cell)) + time_offset

            editor.setMinimum(min_time)
            editor.setMaximum(max_time)
        # Position
        elif column == 1:
            editor.setMinimum(math.degrees(self.min_position))
            editor.setMaximum(math.degrees(self.max_position))
        # Velocity
        elif column == 2:
            editor.setMinimum(math.degrees(-self.velocity_limit))
            editor.setMaximum(math.degrees(self.velocity_limit))

        editor.setDecimals(self.digits)
        editor.setButtonSymbols(QAbstractSpinBox.NoButtons)
        editor.setCorrectionMode(QAbstractSpinBox.CorrectToNearestValue)
        editor.setFixedWidth(75)
        editor.setSingleStep(0)
        editor.setLocale(QtCore.QLocale(QtCore.QLocale.English))

        return editor

    def setEditorData(self, spin_box, index):
        value = float(index.model().data(index, QtCore.Qt.EditRole))

        spin_box.setValue(value)

    def setModelData(self, spin_box, model, index):
        spin_box.interpretText()
        value = round(spin_box.value(), self.digits)

        model.setData(index, str(value), QtCore.Qt.EditRole)

    def updateEditorGeometry(self, editor, option, index):
        editor.setGeometry(option.rect)
