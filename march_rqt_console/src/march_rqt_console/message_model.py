from python_qt_binding.QtCore import QAbstractTableModel, QModelIndex, Qt

from march_rqt_console.message import Message

class MessageModel(QAbstractTableModel):

    columns = ['time', 'level', 'message']

    def __init__(self):
        super(MessageModel, self).__init__()
        self._messages = []
        self._message_limit = 10000

    def rowCount(self, parent=None):
        return len(self._messages)

    def columnCount(self, parent=None):
        return len(MessageModel.columns)

    def headerData(self, section, orientation, role=None):
        if role != Qt.DisplayRole:
            return None
        if orientation == Qt.Horizontal:
            return MessageModel.columns[section].capitalize()

    def data(self, index, role=Qt.DisplayRole):
        column = index.column()
        row = index.row()

        if row >= 0 and row < self.rowCount():
            msg = self._messages[row]

            if role == Qt.DisplayRole:
                if column == 1:
                    return Message.LEVEL_LABELS[msg.level]
                elif column == 2:
                    return msg.message

        return None;

    def insert_row(self, message):
        self.beginInsertRows(QModelIndex(), self.rowCount(), self.rowCount())
        self._messages.append(message)
        self.endInsertRows()
