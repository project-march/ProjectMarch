"""Author: ???."""

from enum import Enum
from march_shared_msgs.msg import GaitInstruction, GaitInstructionResponse


class TransitionRequest(Enum):
    """Enum for transition requests."""

    NONE = 0
    DECREASE_SIZE = -1
    INCREASE_SIZE = 1


class StateMachineInput:
    """Handles input to the state machine.

    Args:
        node (Node): node used to create subscribers/publishers on
    Attributes:
        _logger (Logger): used to log to the terminal
        _stopped (bool): ???
        _paused (bool): ???
        _unknown (bool): ???
        _transition_index (int): ???
        _gait (???): ???
        _node (Node): node to create subscribers/publishers on
        _instruction_subscriber (Subscriber): subscribes to gait_instructions on /march/input_device/instruction
        _instruction_response_publisher (Publisher): publishes response to instruction on
            /march/input_device/instruction_response
    """

    def __init__(self, node):
        self._stopped = False
        self._paused = False
        self._unknown = False
        self._transition_index = 0
        self._gait = None
        self._node = node
        self._logger = node.get_logger().get_child(__class__.__name__)
        self._instruction_subscriber = node.create_subscription(
            msg_type=GaitInstruction,
            topic="/march/input_device/instruction",
            callback=self._callback_input_device_instruction,
            qos_profile=10,
        )
        self._instruction_response_publisher = node.create_publisher(
            msg_type=GaitInstructionResponse,
            topic="/march/input_device/instruction_response",
            qos_profile=20,
        )

    def get_transition_request(self) -> TransitionRequest:
        """Used to return the transition request as an enum.

        Returns:
             TransitionRequest: enum that returns 0, -1, or 1
        """
        if self._transition_index == GaitInstruction.INCREMENT_STEP_SIZE:
            return TransitionRequest.INCREASE_SIZE

        if self._transition_index == GaitInstruction.DECREMENT_STEP_SIZE:
            return TransitionRequest.DECREASE_SIZE

        return TransitionRequest.NONE

    def stop_requested(self) -> bool:
        """Returns True when the current gait should stop, otherwise False."""
        return self._stopped

    def pause_requested(self) -> bool:
        """Returns True when the input requests to pause the current gait, otherwise False."""
        return self._paused

    def unknown_requested(self) -> bool:
        """Returns True when the input requests to transition to the UNKNOWN state, otherwise False."""
        return self._unknown

    def transition_requested(self) -> bool:
        """Returns True when the input requests a gait transition, otherwise False."""
        return self._transition_index != 0

    def gait_requested(self) -> bool:
        """Returns True when the input has a gait selected, otherwise False."""
        return self._gait is not None

    def gait_name(self) -> str:
        """Returns a name of the gait that has been selected, if one was selected, otherwise None."""
        return self._gait

    def reset(self) -> None:
        """Resets the input state to its original state on init."""
        self._stopped = False
        self._paused = False
        self._unknown = False
        self._transition_index = 0
        self._gait = None

    def stop_accepted(self) -> None:
        """If stop is accepted."""
        self._stopped = False

    def stop_rejected(self) -> None:
        """If stop is requested."""
        self._stopped = False

    def gait_accepted(self) -> None:
        """Callback called when the state machine accepts the requested gait."""
        response = GaitInstructionResponse(result=GaitInstructionResponse.GAIT_ACCEPTED)
        self._instruction_response_publisher.publish(response)
        self.reset()

    def gait_rejected(self) -> None:
        """Callback called when the state machine rejects the requested gait."""
        response = GaitInstructionResponse(result=GaitInstructionResponse.GAIT_REJECTED)
        self._instruction_response_publisher.publish(response)
        self.reset()

    def gait_finished(self) -> None:
        """Callback called when the state machine finishes a gait."""
        response = GaitInstructionResponse(result=GaitInstructionResponse.GAIT_FINISHED)
        self._instruction_response_publisher.publish(response)
        self.reset()

    def _callback_input_device_instruction(self, msg: GaitInstruction) -> None:
        """Callback for each new GaitInstruction message.

        Args:
            msg (GaitInstruction): message containing the instruction for the state machine
        """
        self._logger.debug(f"Callback input device instruction {msg}")
        if msg.type == GaitInstruction.STOP:
            self._stopped = True
        elif msg.type == GaitInstruction.GAIT:
            self._gait = msg.gait_name
        elif msg.type == GaitInstruction.PAUSE:
            self._paused = True
        elif msg.type == GaitInstruction.CONTINUE:
            self._paused = False
        elif msg.type == GaitInstruction.INCREMENT_STEP_SIZE:
            self._transition_index = GaitInstruction.INCREMENT_STEP_SIZE
        elif msg.type == GaitInstruction.DECREMENT_STEP_SIZE:
            self._transition_index = GaitInstruction.DECREMENT_STEP_SIZE
        elif msg.type == GaitInstruction.UNKNOWN:
            self._unknown = True
