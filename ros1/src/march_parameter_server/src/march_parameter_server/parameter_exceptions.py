class InvalidParamName(Exception):
    """Custom exception for when a parameter is not available in the parameter server."""

    def __init__(self, name):
        self.name = name

    def __str__(self):
        return "Param with name %s doesn't exist in the parameter server" % self.name
