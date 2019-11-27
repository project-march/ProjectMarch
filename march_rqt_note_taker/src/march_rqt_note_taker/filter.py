from rosgraph_msgs.msg import Log


class Filter:
    """
    Filter class that can add custom filters for accepting
   `rosgraph_msgs.msg.Log`.
    """

    def __init__(self):
        self._filters = []

    def add_filter(self, filter):
        """Adds a filter to accept messages by.

        :type filter: collections.abc.Callable
        :param filter: Filter method that accepts a `rosgraph_msgs.msg.Log` and
                       returns True when the message should be accepted
                       and returns False otherwise
        """
        self._filters.append(filter)

    def add_filter_on_level(self, filter, level):
        self._filters.append(lambda l: filter(l) if l.level == level else False)

    def add_filter_info_level(self, filter):
        self.add_filter_on_level(filter, Log.INFO)

    def apply(self, log_msg):
        """Filters a ROS log msg based on the given filters.

        :type log_msg: rosgraph_msgs.msg.Log
        :param log_msg: Log msg to filter
        :return True when the message is accepted by at least
                one include filter, False otherwise
        """
        for f in self._filters:
            if f(log_msg):
                return True

        return False
