import os

from python_qt_binding import QtCore, loadUi
from python_qt_binding.QtCore import QAbstractTableModel
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QFileDialog, QShortcut, QWidget

from rclpy.node import Node

from .entry import Entry


class NotesWidget(QWidget):
    INSERT = 0
    REMOVE = 1

    def __init__(self, model: QAbstractTableModel, ui_file: str, node: Node):
        """Initialize the NotesWidget.

        :param model
        :param ui_file Path to ui file.
        :param node Handle to a node, used for logging
        """
        super(NotesWidget, self).__init__()

        self._model = model
        self._can_save = True
        self._has_autosave = True
        self._autosave_file = None
        self._last_save_file = None

        self._node = node

        loadUi(ui_file, self)

        self._model.rowsInserted.connect(
            lambda parent, first, last: [
                self.update_status(),
                self._set_saved(False),
                self._autosave(first, last, self.INSERT),
            ]
        )
        self._model.rowsRemoved.connect(
            lambda parent, first, last: [
                self.update_status(),
                self._set_saved(False),
                self._autosave(first, last, self.REMOVE),
            ]
        )

        self.table_view.setModel(self._model)
        self.table_view.verticalScrollBar().rangeChanged.connect(
            self._handle_change_scroll
        )
        self._last_scroll_max = self.table_view.verticalScrollBar().maximum()

        self.input_field.returnPressed.connect(self._handle_insert_entry)

        self.take_button.clicked.connect(self._handle_start_take)

        self.load_button.clicked.connect(self._handle_load)
        self.save_button.clicked.connect(self._handle_save)
        self.autosave_check_box.stateChanged.connect(self._handle_autosave)
        self.autosave_check_box.setChecked(self._has_autosave)
        self.autosave_check_box.setEnabled(False)

        self._delete_shortcut = QShortcut(QKeySequence("Delete"), self)
        self._delete_shortcut.activated.connect(self._delete_selected)

        self._load_shortcut = QShortcut(QKeySequence("Ctrl+O"), self)
        self._load_shortcut.activated.connect(self._handle_load)

        self._save_shortcut = QShortcut(QKeySequence("Ctrl+S"), self)
        self._save_shortcut.activated.connect(self._handle_save)

    def _handle_insert_entry(self):
        """Callback for when an entry is inserted."""
        entry = self.input_field.text().strip()
        if entry:
            self._model.insert_row(Entry(entry))
        self.input_field.clear()
        self.table_view.verticalScrollBar().setSliderPosition(self._last_scroll_max)

    def update_status(self):
        """Update status when a row is added or removed.

        The status display tells the user the total amount of messages is updated.
        """
        self.messages_label.setText(f"Displaying {len(self._model)} messages")

    def _handle_start_take(self):
        """Callback for when the 'Start take' button is pressed."""
        take = self.camera_spin_box.value()
        self._model.insert_row(Entry(f"Started camera take {take}"))
        self.camera_spin_box.setValue(take + 1)
        self.table_view.verticalScrollBar().setSliderPosition(self._last_scroll_max)

    def _handle_change_scroll(self, scroll_min, scroll_max):
        """Callback for when the mouse is scrolled."""
        if self.table_view.verticalScrollBar().value() == self._last_scroll_max:
            self.table_view.verticalScrollBar().setSliderPosition(scroll_max)
            self._last_scroll_max = scroll_max

    def _delete_selected(self):
        """Callback for when the delete button is pressed.

        All selected rows are deleted from the model.
        """
        selection_model = self.table_view.selectionModel()
        if self.table_view.hasFocus() and selection_model.hasSelection():
            indices = [
                index
                for index in selection_model.selectedIndexes()
                if not index.column()
            ]
            if indices and all([index.isValid() for index in indices]):
                self._model.remove_rows([index.row() for index in indices])

    def _set_saved(self, saved):
        self._can_save = not saved
        self.save_button.setEnabled(not saved)

    def _handle_load(self):
        """Callback for when the load button is pressed.

        Not yet implemented.
        """
        self._node.get_logger().warn("Loading notes from a file is not yet implemented")

    def _handle_save(self):
        """Callback for when the save button is pressed."""
        result = QFileDialog.getSaveFileName(
            self, "Save File", ".", "Minute files (*.txt)"
        )
        file_name = result[0]
        if file_name:
            self._save(file_name)

    def _handle_autosave(self, state):
        """If the autosave checkbox is marked, any new entries are
        automatically added to the saved file"""
        self._has_autosave = state == QtCore.Qt.Checked
        if not self._has_autosave and self._autosave_file is not None:
            self._autosave_file.close()
        elif self._has_autosave and self._can_save and self._last_save_file:
            self._save(self._last_save_file)

    def _autosave(self, first: int, last: int, action: int):
        """Write the last changes incrementally to the autosave file.

        :param first: The first index of affected entries in the model
        :param last: the last index of affected entries in the model(inclusive)
        :param action: Either INSERT or REMOVE
        """
        if self._has_autosave and self._last_save_file is not None:
            if self._autosave_file is None or self._autosave_file.closed:
                try:
                    self._autosave_file = open(self._last_save_file, "r+")
                    self._autosave_file.seek(0, os.SEEK_END)
                except IOError as err:
                    self._node.get_logger().error(
                        f"Failed to open file {self._last_save_fil} "
                        f"for autosaving: {err}"
                    )
                    return
            try:
                if action == self.INSERT:
                    if first != 0:
                        self._autosave_file.write("\n")
                    self._autosave_file.write(
                        "\n".join(str(entry) for entry in self._model[first : last + 1])
                    )
                elif action == self.REMOVE:
                    self._autosave_file.seek(0, os.SEEK_SET)
                    self._autosave_file.write(str(self._model))
                    self._autosave_file.truncate()
                else:
                    self._node.get_logger().warn(f"Unknown autosave action: {action}")
                    return
                self._autosave_file.flush()
            except IOError as err:
                self._node.get_logger().error(
                    f"Failed to write to file {self._last_save_fil} "
                    f"for autosaving: {err}"
                )
            else:
                self._set_saved(True)

    def _save(self, file_name: str):
        """Save the model contents to a file.

        :param file_name Name of the file
        """
        if file_name[-4:] != ".txt":
            file_name += ".txt"
        try:
            with open(file_name, "w") as f:
                f.write(str(self._model))
        except IOError as err:
            self._node.get_logger().error(f"Failed to open file: {err}")
        else:
            self._set_saved(True)
            self._last_save_file = file_name
            self.autosave_check_box.setEnabled(True)
