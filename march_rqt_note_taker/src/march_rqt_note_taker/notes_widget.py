import os

from python_qt_binding import loadUi, QtCore
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QFileDialog, QShortcut, QWidget
import rospy

from .entry import Entry


class NotesWidget(QWidget):
    INSERT = 0
    REMOVE = 1

    def __init__(self, model, ui_file):
        super(NotesWidget, self).__init__()

        self._model = model
        self._can_save = True
        self._has_autosave = True
        self._autosave_file = None
        self._last_save_file = None

        loadUi(ui_file, self)

        self._model.rowsInserted.connect(
            lambda parent, first, last: [self.update_status(), self._set_saved(False),
                                         self._autosave(first, last, self.INSERT)])
        self._model.rowsRemoved.connect(
            lambda parent, first, last: [self.update_status(), self._set_saved(False),
                                         self._autosave(first, last, self.REMOVE)])

        self.table_view.setModel(self._model)
        self.table_view.verticalScrollBar().rangeChanged.connect(self._handle_change_scroll)
        self._last_scroll_max = self.table_view.verticalScrollBar().maximum()

        self.input_field.returnPressed.connect(self._handle_insert_entry)

        self.take_button.clicked.connect(self._handle_start_take)

        self.load_button.clicked.connect(self._handle_load)
        self.save_button.clicked.connect(self._handle_save)
        self.autosave_check_box.stateChanged.connect(self._handle_autosave)
        self.autosave_check_box.setChecked(self._has_autosave)
        self.autosave_check_box.setEnabled(False)

        self._delete_shortcut = QShortcut(QKeySequence('Delete'), self)
        self._delete_shortcut.activated.connect(self._delete_selected)

        self._load_shortcut = QShortcut(QKeySequence('Ctrl+O'), self)
        self._load_shortcut.activated.connect(self._handle_load)

        self._save_shortcut = QShortcut(QKeySequence('Ctrl+S'), self)
        self._save_shortcut.activated.connect(self._handle_save)

    def _handle_insert_entry(self):
        entry = self.input_field.text().strip()
        if entry:
            self._model.insert_row(Entry(entry))
        self.input_field.clear()
        self.table_view.verticalScrollBar().setSliderPosition(self._last_scroll_max)

    def update_status(self):
        self.messages_label.setText('Displaying {0} messages'.format(len(self._model)))

    def _handle_start_take(self):
        take = self.camera_spin_box.value()
        self._model.insert_row(Entry('Started camera take {0}'.format(take)))
        self.camera_spin_box.setValue(take + 1)
        self.table_view.verticalScrollBar().setSliderPosition(self._last_scroll_max)

    def _handle_change_scroll(self, scroll_min, scroll_max):
        if self.table_view.verticalScrollBar().value() == self._last_scroll_max:
            self.table_view.verticalScrollBar().setSliderPosition(scroll_max)
            self._last_scroll_max = scroll_max

    def _delete_selected(self):
        selection_model = self.table_view.selectionModel()
        if self.table_view.hasFocus() and selection_model.hasSelection():
            indices = [index for index in selection_model.selectedIndexes() if not index.column()]
            if indices and indices[0].isValid():
                self._model.remove_rows(indices[0].row(), len(indices))

    def _set_saved(self, saved):
        self._can_save = not saved
        self.save_button.setEnabled(not saved)

    def _handle_load(self):
        rospy.logwarn('Loading notes from a file is not yet implemented')

    def _handle_save(self):
        result = QFileDialog.getSaveFileName(self, 'Save File', '.', 'Minute files (*.txt)')
        file_name = result[0]
        if file_name:
            self._save(file_name)

    def _handle_autosave(self, state):
        self._has_autosave = state == QtCore.Qt.Checked
        if not self._has_autosave and self._autosave_file is not None:
            self._autosave_file.close()
        elif self._has_autosave and self._can_save and self._last_save_file:
            self._save(self._last_save_file)

    def _autosave(self, first, last, action):
        """Writes the last changes incrementally to the autosave file.

        :param int first: The first index of affected entries in the model
        :param int last: the last index of affected entries in the model (inclusive)
        :param int action: Either INSERT or REMOVE
        """
        if self._has_autosave and self._last_save_file is not None:
            if self._autosave_file is None or self._autosave_file.closed:
                try:
                    self._autosave_file = open(self._last_save_file, 'r+')
                    self._autosave_file.seek(0, os.SEEK_END)
                except IOError as e:
                    rospy.logerr('Failed to open file {f} for autosaving: {e}'.format(f=self._last_save_file, e=e))
                    return

            try:
                if action == self.INSERT:
                    if first != 0:
                        self._autosave_file.write('\n')
                    self._autosave_file.write('\n'.join(str(entry) for entry in self._model[first:last + 1]))
                elif action == self.REMOVE:
                    self._autosave_file.seek(0, os.SEEK_SET)
                    self._autosave_file.write(str(self._model))
                    self._autosave_file.truncate()
                else:
                    rospy.logwarn('Unknown autosave action: {a}'.format(a=action))
                    return
                self._autosave_file.flush()
            except IOError as e:
                rospy.logerr('Failed to write to file {f} for autosaving: {e}'.format(f=self._last_save_file, e=e))
            else:
                self._set_saved(True)

    def _save(self, file_name):
        if file_name[-4:] != '.txt':
            file_name += '.txt'
        try:
            with open(file_name, 'w') as f:
                f.write(str(self._model))
        except IOError as e:
            rospy.logerr('Failed to open file: {e}'.format(e=e))
        else:
            self._set_saved(True)
            self._last_save_file = file_name
            self.autosave_check_box.setEnabled(True)
