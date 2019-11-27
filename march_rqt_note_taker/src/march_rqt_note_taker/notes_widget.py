from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from .entry import Entry


class NotesWidget(QWidget):

    def __init__(self, model, ui_file):
        super(NotesWidget, self).__init__()

        self._model = model

        loadUi(ui_file, self)

        self._model.rowsInserted.connect(self.update_status)
        self._model.rowsRemoved.connect(self.update_status)

        self.table_view.setModel(self._model)
        self.table_view.verticalScrollBar().rangeChanged.connect(self._handle_change_scroll)
        self._last_scroll_max = self.table_view.verticalScrollBar().maximum()

        self.input_field.returnPressed.connect(self._handle_insert_entry)

        self.take_button.clicked.connect(self._handle_start_take)

    def _handle_insert_entry(self):
        entry = self.input_field.text().strip()
        if entry:
            self._model.insert_row(Entry(entry))
        self.input_field.clear()
        self.table_view.verticalScrollBar().setSliderPosition(self._last_scroll_max)

    def update_status(self):
        self.messages_label.setText('Displaying {} messages'.format(self._model.rowCount()))

    def _handle_start_take(self):
        take = self.camera_spin_box.value()
        self._model.insert_row(Entry('Started camera take {}'.format(take)))
        self.table_view.verticalScrollBar().setSliderPosition(self._last_scroll_max)

    def _handle_change_scroll(self, scroll_min, scroll_max):
        if self.table_view.verticalScrollBar().value() == self._last_scroll_max:
            self.table_view.verticalScrollBar().setSliderPosition(scroll_max)
            self._last_scroll_max = scroll_max
