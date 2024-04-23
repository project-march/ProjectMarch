class IPD:
    def __init__(self):
        self._current_mode = None
        self._available_modes = set()

    def get_current_mode(self):
        return self._current_mode

    def get_available_modes(self):
        return self._available_modes

    def ask_new_mode(self):
        # Implement the logic to ask for a new mode here
        pass

    def set_current_mode(self, current_mode):
        self._current_mode = current_mode

    def set_available_modes(self, available_modes):
        self._available_modes = available_modes