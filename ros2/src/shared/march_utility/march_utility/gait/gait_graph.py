from typing import Dict, Set, List

from march_gait_selection.gait_selection import GaitSelection
from march_gait_selection.state_machine.gait_state_machine_error import (
    GaitStateMachineError,
)
from march_gait_selection.gaits.home_gait import HomeGait
from march_utility.gait.edge_position import DynamicEdgePosition

from .edge_position import EdgePosition, StaticEdgePosition, UnknownEdgePosition


class GaitGraph:
    """
    The gait graph is made to keep track of all named positions and gait transitions
    that are available. It is generated at startup and is used by the gait state
    machine to get the possible gaits and to verify a requested gait.
    """

    NamedPositions = Dict[EdgePosition, str]
    IdleTransitions = Dict[EdgePosition, Set[str]]
    GaitTransitions = Dict[str, EdgePosition]

    UNNAMED = "unnamed"
    UNKNOWN = "unknown"

    def __init__(self, gait_selection: GaitSelection):
        self._gait_selection = gait_selection

        self._named_positions: GaitGraph.NamedPositions = {}
        self._idle_transitions: GaitGraph.IdleTransitions = {}
        self._dynamic_transitions: Set = set()
        self._gait_transitions: GaitGraph.GaitTransitions = {}
        self._unnamed_count = 0

    def possible_gaits_from_idle(self, current_state: EdgePosition) -> Set:
        """
        Get the list of gait names that are (possibly) available from this edge
        position.
        :param current_state: The current position of the exo.
        :return: A list of gait names
        """
        if isinstance(current_state, DynamicEdgePosition):
            return self._dynamic_transitions
        else:
            return self._idle_transitions[current_state]

    def generate_graph(self):
        """Generate the gait graph."""
        self._make_named_positions()
        self._make_transitions()
        self._make_home_gaits()
        self._validate_from_transitions()
        self._validate_to_transitions()

    def _make_named_positions(self):
        """Create all named positions from the default.yaml.

        Create a StaticEdgePosition for every named position and a default UnknownEdgePosition.
        """
        self._named_positions = {
            StaticEdgePosition(position["joints"]): name
            for name, position in self._gait_selection.positions.items()
        }
        self._named_positions[UnknownEdgePosition()] = GaitGraph.UNKNOWN

    def get_name_of_position(self, position: EdgePosition) -> str:
        """
        Get the name of a given position. Static positions will have a name from the
        default.yaml or a name generated at startup. Dynamic gaits have no name,
        but return a string with the values.
        :param position: The position to get the name from.
        :return: The name.
        """
        if position in self._named_positions.keys():
            return self._named_positions[position]
        else:
            return f"unnamed: {position}"

    def _make_transitions(self):
        """Make all gait transitions.

        For every gait, first checks the starting position and adds an idle transition
        from the gait's starting position to the gait name.
        Then the final position is checked, and a transition from the gait name
        to the final position is added.
        """
        for gait in self._gait_selection._gaits.values():
            if gait.starting_position not in self._named_positions:
                if isinstance(gait.starting_position, StaticEdgePosition):
                    position_name = self._new_unnamed()
                    self._gait_selection.get_logger().warn(
                        f"No named position given for starting position of gait `"
                        f"{gait.name}, creating {position_name}."
                    )
                    self._named_positions[gait.starting_position] = position_name
            self._add_idle_transition(gait.starting_position, gait.gait_name)

            if gait.final_position not in self._named_positions:
                if isinstance(gait.final_position, StaticEdgePosition):
                    position_name = self._new_unnamed()
                    self._gait_selection.get_logger().warn(
                        f"No named position given for final position of gait `"
                        f"{gait.name}, creating {position_name}"
                    )
                    self._named_positions[gait.final_position] = position_name
            self._gait_transitions[gait.gait_name] = gait.final_position

    def _make_home_gaits(self):
        """Make all home gaits.

        For every named position, create a home gait and add the home gait
        to the GaitSelection and to the idle transitions.
        """
        for position, name in self._named_positions.items():
            if isinstance(position, UnknownEdgePosition):
                continue
            position_dict = dict(zip(self._gait_selection.joint_names, position.values))
            home_gait = HomeGait(name, position_dict, "")
            home_gait_name = home_gait.name

            if home_gait_name in self._gait_transitions:
                raise GaitStateMachineError(
                    f"Gaits cannot have the same name as home gait `{home_gait_name}`"
                )

            self._gait_selection._gaits[home_gait_name] = home_gait
            self._add_idle_transition(home_gait.starting_position, home_gait_name)

    def _add_idle_transition(self, start_position: EdgePosition, gait_name: str):
        """Add an idle transition.

        :param start_position Position the gait is started from.
        :param gait_name Name of the gait that is started.
        """
        if isinstance(start_position, DynamicEdgePosition):
            self._dynamic_transitions.add(gait_name)
        elif start_position in self._idle_transitions:
            self._idle_transitions[start_position].add(gait_name)
        else:
            self._idle_transitions[start_position] = {gait_name}

    def _validate_from_transitions(self):
        """Check that from every position there is a transition

        :return Returns true if from every position there is a transition.
        """
        no_from_transitions = []
        for position, name in self._named_positions.items():
            if position not in self._idle_transitions:
                no_from_transitions.append(name)
        if len(no_from_transitions) > 0:
            self._gait_selection.get_logger().warn(
                f'There are no transitions from named positions: [{", ".join(no_from_transitions)}]'
            )
            return False
        return True

    def _validate_to_transitions(self):
        """Check that all positions can be reached.

        :return Returns true if all positions can be reached.
        """
        no_to_transitions = []
        for position, name in self._named_positions.items():
            if (
                not isinstance(position, UnknownEdgePosition)
                and position not in self._gait_transitions.values()
            ):
                no_to_transitions.append(name)

        if len(no_to_transitions) > 0:
            self._gait_selection.get_logger().warn(
                f'There are no transitions to named positions: [{", ".join(no_to_transitions)}]'
            )
            return False
        return True

    def _new_unnamed(self) -> str:
        """Generate a new unnamed position.

        The first time this will give the position unnamed_0.
        The unnamed count is increased each call.
        """
        name = f"{GaitGraph.UNNAMED}_{self._unnamed_count}"
        self._unnamed_count += 1
        return name

    def __str__(self) -> str:
        """Convert the gait graph to a human-friendly string."""
        s = "Positions:\n"
        for position, name in self._named_positions.items():
            s += f"\t{name}: {position}\n"
        s += "Idle transitions:\n"
        for position, gaits in self._idle_transitions.items():
            for gait in gaits:
                s += f"\t{self._named_positions[position]} - {gait}\n"
        s += "Gait transitions:\n"
        for gait, position in self._gait_transitions.items():
            s += f"\t{gait} - {self._named_positions[position]}\n"

        return s
