"""Author: Olav de Haas, MV."""
from collections import deque
from typing import List

from march_utility.exceptions.gait_exceptions import SubgaitGraphError


class SubgaitGraph:
    """To validate and check the transitions of all subgaits."""

    START = "start"
    END = "end"
    TO = "to"
    STOP = "stop"
    INCREASE_SIZE = "increase_size"
    DECREASE_SIZE = "decrease_size"
    TRANSITIONS = [TO, STOP, INCREASE_SIZE, DECREASE_SIZE]

    def __init__(self, graph: dict):
        self._graph = graph
        self._stoppable = False
        self.validate()

    def validate(self) -> None:
        """Validates the graph and raises an exception when not valid.

        This method checks a few things to prove consistency:
            1. Checks that a `start` and `end` state exist
            2. Checks that is possible to get to every state from `start`
            3. Checks that it is possible to get from every state to `end`
            4. Checks that all subgaits do not have equal `stop` and `to` transitions

        Raises:
            SubgaitGraphError. If:
                * There is no `start` state.
                * One of the gaits is not in the gait `graph`.
                * One of the gaits does not have a (known) transition to another gait.
                * The transition to `start` state is not allowed.
                * There is no `end` state.
                * There is no way to get to the `end` state.

        """
        if self.START not in self._graph:
            raise SubgaitGraphError("There is no state {s}".format(s=self.START))

        # Do a breadth-first search to check for validity
        queue = deque([(self.START, set())])
        visited = {}
        while len(queue) != 0:
            name, from_subgaits = queue.popleft()
            if name in visited:
                visited[name] = visited[name].union(from_subgaits)
                continue

            visited[name] = from_subgaits
            if name == self.END:
                continue

            self._validate_subgait(name)
            subgait = self._graph[name]
            for transition in self.TRANSITIONS:
                if transition in subgait:
                    from_subgaits = from_subgaits.copy()
                    from_subgaits.add(name)
                    queue.append((subgait[transition], from_subgaits))
                    if transition == self.STOP:
                        self._stoppable = True

        self._validate_visited(visited)

    def _validate_subgait(self, name: str) -> None:
        """Checks if the subgait itself is valid.

        Raises:
            SubgaitGraphError. If:
                * The gaits is not in the gait `graph`.
                * The gaits does not have a (known) transition to another gait.
        """
        subgait = self._graph.get(name)
        if subgait is None:
            raise SubgaitGraphError("Subgait {n} is not a subgait in the graph".format(n=name))
        if self.TO not in subgait:
            raise SubgaitGraphError("Subgait {n} has no `{t}` transition".format(n=name, t=self.TO))
        if not all(transition in self.TRANSITIONS for transition in subgait):
            raise SubgaitGraphError(
                "Subgait {n} has unknown transitions. Available transitions {t}".format(n=name, t=self.TRANSITIONS)
            )
        if len(set(subgait.values())) != len(subgait):
            raise SubgaitGraphError("Subgait {n} transitions cannot be equal".format(n=name))

    def _validate_visited(self, visited) -> None:
        """Checks if the there is a valid path through all subgaits.

        Raises:
            SubgaitGraphError. If:
                * The transition to `start` state is not allowed.
                * There is no `end` state.
                * There is no way to get to the `end` state.
        """
        if len(visited[self.START]) != 0:
            raise SubgaitGraphError("Transition to `{s}` is not allowed".format(s=self.START))
        if self.END not in visited:
            raise SubgaitGraphError("There are no transitions to `{e}`".format(e=self.END))
        if len(visited[self.END]) != len(self._graph):
            not_covered = set(self._graph).difference(visited[self.END])
            raise SubgaitGraphError("`{e}` is not reachable from {s}".format(e=self.END, s=not_covered))

    def is_stoppable(self) -> bool:
        """Returns True when the graph contains a stop transition, False otherwise."""
        return self._stoppable

    def start_subgaits(self) -> List[str]:
        """Returns a list of subgait names that transition from the `start` state."""
        return list(self._graph[self.START].values())

    def end_subgaits(self) -> List[str]:
        """Returns a list of subgait names that transition to the `end` state."""
        return [from_subgait for from_subgait, transitions in self._graph.items() if self.END in transitions.values()]

    @property
    def graph(self):
        """dict. Returns the subgait graph as a dictionary."""
        return self._graph

    def __contains__(self, subgait_name):
        """Checks if the given subgait name is contained in the graph."""
        return subgait_name in self._graph

    def __getitem__(self, transition):
        """Returns the subgait the given subgait transitions to.

        :Args:
            transition (str, str): Tuple of subgait name and type of transition, can be either 'to' or 'stop'.

        Returns:
            str. Name of subgait that the given transition transitions to.
        """
        subgait_name, transition_type = transition
        if subgait_name not in self._graph:
            raise KeyError("Gait does not contain subgait {0}".format(subgait_name))
        return self._graph[subgait_name].get(transition_type)

    def __iter__(self):
        """Returns an iterator over all possible transitions in arbitrary order.

        Excludes size transitions.
        """
        return iter(
            [
                (from_subgait, to_subgait)
                for from_subgait, transitions in self._graph.items()
                for transition, to_subgait in transitions.items()
                if transition not in {self.INCREASE_SIZE, self.DECREASE_SIZE}
            ]
        )

    def __eq__(self, other) -> bool:
        """Checks if the other is a subgait and has the same subgait graph."""
        return isinstance(other, SubgaitGraph) and self._graph == other._graph
