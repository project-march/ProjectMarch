"""Author: ???."""


class GaitStateMachineError(Exception):
    """A base class for any errors in the state machine.

    Args:
        msg (:obj: str, optional): message to return with error. Gets appended to "An error occurred in the gait
            state machine"
    """

    def __init__(self, msg=None) -> None:
        base_msg = "An error occurred in the gait state machine"
        if msg is not None:
            base_msg += ": " + msg
        super(GaitStateMachineError, self).__init__(msg)
