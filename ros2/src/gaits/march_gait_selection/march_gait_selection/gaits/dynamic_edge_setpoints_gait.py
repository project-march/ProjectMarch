from march_gait_selection.gaits.setpoints_gait import SetpointsGait
from march_utility.gait.edge_position import DynamicEdgePosition


class DynamicEdgeSetpointsGait(SetpointsGait):
    """ The standard gait built up from setpoints, but with dynamic edge positions.
    This is mostly useful for debugging realsense gaits and testing this without
    restarting the state machine. """

    def __init__(self, gait_name, subgaits, graph):
        super(DynamicEdgeSetpointsGait, self).__init__(gait_name, subgaits, graph)
        self._starting_position = DynamicEdgePosition(self.subgaits[
                                                   self.graph.start_subgaits()[0]].starting_position)
        self._final_position = DynamicEdgePosition(self.subgaits[
                                                     self.graph.end_subgaits()[
            0]].final_position)

    @property
    def starting_position(self):
        return self._starting_position

    @property
    def final_position(self):
        return self._final_position