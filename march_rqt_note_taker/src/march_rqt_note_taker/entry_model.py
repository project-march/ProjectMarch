from python_qt_binding.QtCore import QAbstractTableModel, QModelIndex, Qt
from python_qt_binding.QtGui import QBrush

from .entry import Entry


class EntryModel(QAbstractTableModel):

    columns = ['time', 'entry']

    def __init__(self):
        super(EntryModel, self).__init__()
        self._entries = []

    def rowCount(self, parent=None):
        return len(self._entries)

    def columnCount(self, parent=None):
        return len(EntryModel.columns)

    def headerData(self, section, orientation, role=None):
        if orientation == Qt.Horizontal:
            if role == Qt.DisplayRole:
                return EntryModel.columns[section].capitalize()

    def data(self, index, role=Qt.DisplayRole):
        column = index.column()
        row = index.row()

        if 0 <= row < self.rowCount() and 0 <= column < self.columnCount():
            column = EntryModel.columns[index.column()]
            entry = self._entries[row]

            if role == Qt.DisplayRole:
                if column == 'time':
                    return entry.time_string()
                elif column == 'entry':
                    return entry.content

            if role == Qt.ForegroundRole:
                if column == 'entry' and entry.is_error:
                    return QBrush(Qt.darkRed)

        return None

    def remove_rows(self, position, rows=1):
        """Removes the rows with given indices.

        :type position: int
        :param position: Start position to remove
        :type rows: int
        :param rows: Amount of rows to remove
        :returns True when the removal was successful
        """
        self.beginRemoveRows(QModelIndex(), position, position + rows - 1)
        self._entries = self._entries[:position] + self._entries[position + rows:]
        self.endRemoveRows()

    def insert_row(self, entry):
        """Appends an entry.

        :type entry: Entry
        :param entry: Entry to append to rows
        """
        self.beginInsertRows(QModelIndex(), self.rowCount(), self.rowCount())
        self._entries.append(entry)
        self.endInsertRows()

    def insert_log_msg(self, log_msg):
        """Converts a ROS log msg to entry and appends it to the rows.

        :type log_msg: rosgraph_msgs.msg.Log
        :param log_msg: Log msg to
        """
        self.insert_row(Entry.from_ros_msg(log_msg))

    def __str__(self):
        """Returns a string representation of the model."""
        return '\n'.join(str(x) for x in self._entries)
