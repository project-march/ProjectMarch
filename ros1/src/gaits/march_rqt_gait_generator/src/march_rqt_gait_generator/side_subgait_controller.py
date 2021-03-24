from numpy_ringbuffer import RingBuffer


class SideSubgaitController(object):
    def __init__(
        self,
        default,
        view,
        lock_checked=False,
        default_checked=False,
        subgait=None,
    ):
        self._lock_checked = lock_checked
        self._default_checked = default_checked
        self._subgait = subgait
        self._default = default
        self.view = view

        self.settings_history = RingBuffer(capacity=100, dtype=list)
        self.settings_redo_list = RingBuffer(capacity=100, dtype=list)

    def undo(self):
        self.settings_redo_list.append(
            {
                "lock_checked": self._lock_checked,
                "default_checked": self._default_checked,
                "subgait": self._subgait,
            }
        )
        settings = self.settings_history.pop()
        self._lock_checked = settings["lock_checked"]
        self._default_checked = settings["default_checked"]
        self._subgait = settings["subgait"]
        self.view.update_widget(self)

    def redo(self):
        self.save_changed_settings()
        settings = self.settings_redo_list.pop()
        self._lock_checked = settings["lock_checked"]
        self._default_checked = settings["default_checked"]
        self._subgait = settings["subgait"]
        self.view.update_widget(self)

    def save_changed_settings(self):
        self.settings_history.append(
            {
                "lock_checked": self._lock_checked,
                "default_checked": self._default_checked,
                "subgait": self._subgait,
            }
        )

    @property
    def lock_checked(self):
        return self._lock_checked

    @lock_checked.setter
    def lock_checked(self, lock_checked):
        self.save_changed_settings()
        self._lock_checked = lock_checked
        self.view.update_widget(self)

    @property
    def default_checked(self):
        return self._default_checked

    @default_checked.setter
    def default_checked(self, default_checked):
        self.save_changed_settings()
        self._default_checked = default_checked
        self.view.update_widget(self)

    @property
    def subgait(self):
        if self.default_checked:
            return self._default
        else:
            return self._subgait

    @subgait.setter
    def subgait(self, subgait):
        self.save_changed_settings()
        self._subgait = subgait
        self.view.update_widget(self)

    @property
    def subgait_text(self):
        if self._subgait:
            return self._subgait.version
        else:
            return "Import"
