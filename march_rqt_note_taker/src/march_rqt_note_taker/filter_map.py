from rosgraph_msgs.msg import Log


class FilterMap:
    """Filter class that can add custom filters and mappings for accepting `rosgraph_msgs.msg.Log`."""

    def __init__(self):
        self._filter_maps = []

    def add_filter(self, msg_filter, msg_map=lambda m: m):
        """Adds a filter to accept messages by and map to transform them.

        :type msg_filter: collections.abc.Callable
        :param msg_filter: Filter method that accepts a `rosgraph_msgs.msg.Log`
                           and returns True when the message should be accepted
                           and returns False otherwise
        :type msg_map: collections.abc.Callable
        :param msg_map: Optional map method that accepts a string
        """
        self._filter_maps.append((msg_filter, msg_map))

    def add_filter_on_level(self, msg_filter, level, msg_map=lambda m: m):
        self._filter_maps.append((
            lambda l: msg_filter(l) if l.level == level else False,
            msg_map,
        ))

    def add_filter_info_level(self, msg_filter, msg_map=lambda m: m):
        self.add_filter_on_level(msg_filter, Log.INFO, msg_map)

    def __call__(self, log_msg):
        """Filters a ROS log msg based on the given filters.

        :type log_msg: rosgraph_msgs.msg.Log
        :param log_msg: Log msg to filter
        :return Mapped message string when the message is accepted by at least
                one include filter, None otherwise
        """
        for (f, m) in self._filter_maps:
            if f(log_msg):
                log_msg.msg = m(log_msg.msg)
                return log_msg

        return None
