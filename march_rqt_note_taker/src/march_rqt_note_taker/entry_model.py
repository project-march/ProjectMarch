from python_qt_binding.QtCore import QAbstractTableModel, QModelIndex, Qt


class EntryModel(QAbstractTableModel):

    columns = ['time', 'entry']

    def __init__(self):
        super(EntryModel, self).__init__()
        self._entries = []
        self._entries_limit = 10000

    def rowCount(self, parent=None):
        return len(self._entries)

    def columnCount(self, parent=None):
        return len(EntryModel.columns)

    def headerData(self, section, orientation, role=None):
        if role != Qt.DisplayRole:
            return None
        if orientation == Qt.Horizontal:
            return EntryModel.columns[section].capitalize()

    def data(self, index, role=Qt.DisplayRole):
        column = index.column()
        row = index.row()

        if 0 <= row < self.rowCount():
            entry = self._entries[row]

            if role == Qt.DisplayRole:
                if column == 0:
                    return entry.time_string()
                elif column == 1:
                    return entry.content

        return None

    def insert_row(self, entry):
        self.beginInsertRows(QModelIndex(), self.rowCount(), self.rowCount())
        self._entries.append(entry)
        self.endInsertRows()
