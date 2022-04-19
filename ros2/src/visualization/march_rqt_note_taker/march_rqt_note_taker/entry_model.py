"""Author: Olav de Haas, MIV."""
from typing import List, Optional

from python_qt_binding.QtCore import QAbstractTableModel, QModelIndex, Qt
from python_qt_binding.QtGui import QBrush

from rcl_interfaces.msg import Log

from .entry import Entry


class EntryModel(QAbstractTableModel):
    """Creates an EntryModel with an empty list of entries."""

    COLUMNS = ["time", "entry"]

    def __init__(self):
        super(EntryModel, self).__init__()
        self._entries: List[Entry] = []

    # Ignore 'N802': function name cannot be snake_case because it
    # overrides a function from QAbstractTableModel
    def rowCount(self, parent=None) -> int:  # noqa: N802
        """Get the number of rows.

        Returns:
            int: The number of rows.
        """
        return len(self._entries)

    # Ignore 'N802': function name cannot be snake_case because it
    # overrides a function from QAbstractTableModel
    def columnCount(self, parent=None) -> int:  # noqa: N802
        """Get the number of columns.

        Return:
             int: The number of columns.
        """
        return len(EntryModel.COLUMNS)

    # Ignore 'N802': function name cannot be snake_case because it
    # overrides a function from QAbstractTableModel
    def headerData(self, section: int, orientation, role=None):  # noqa: N802
        """Returns the header row names, capitalized."""
        if orientation == Qt.Horizontal and role == Qt.DisplayRole:
            return EntryModel.COLUMNS[section].capitalize()
        return None

    def data(self, index, role=Qt.DisplayRole):
        """The data that is in the EntryModel table."""
        column = index.column()
        row = index.row()

        if 0 <= row < self.rowCount() and 0 <= column < self.columnCount():
            column = EntryModel.COLUMNS[index.column()]
            entry = self._entries[row]

            if role == Qt.DisplayRole:
                if column == "time":
                    return entry.time_string()
                elif column == "entry":
                    return entry.content

            if role == Qt.ForegroundRole and column == "entry" and entry.is_error:
                return QBrush(Qt.darkRed)

        return None

    def remove_rows(self, positions: List[int]):
        """Removes the rows with given indices.

        Args:
             positions (List[int]): Positions to remove.
        """
        for row in sorted(positions, reverse=True):
            if 0 <= row < self.rowCount():
                self.beginRemoveRows(QModelIndex(), row, row)
                del self._entries[row]
                self.endRemoveRows()

    def insert_row(self, entry: Entry):
        """Appends an entry.

        Args:
             entry (Entry): Entry to append to rows.
        """
        self.beginInsertRows(QModelIndex(), self.rowCount(), self.rowCount())
        self._entries.append(entry)
        self.endInsertRows()

    def insert_log_msg(self, log_msg: Log, use_current_time: Optional[bool] = True):
        """Converts a ROS log msg to entry and appends it to the rows.

        Args:
            log_msg (Log): Log msg that should be inserted as an entry in the data.
            use_current_time (bool): Whether the current time should be used, instead of the timestamp of the log.
        """
        self.insert_row(Entry.from_ros_msg(log_msg, use_current_time))

    def __str__(self):
        """Returns a string representation of the model."""
        return "\n".join(str(x) for x in self._entries)

    def __len__(self):
        """Returns the amount of data entries there are in the table."""
        return len(self._entries)

    def __getitem__(self, index: int) -> Entry:
        """Returns a data entry at the given index."""
        return self._entries[index]
