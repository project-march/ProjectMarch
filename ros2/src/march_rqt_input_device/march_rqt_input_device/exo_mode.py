"""Author: Andrew Hutani, MIX"""

from enum import Enum

class ExoMode(Enum):
    Sit = 0
    Stand = 1
    Walk = 2
    BootUp = 3
    Error = 4
    Sideways = 5
    LargeWalk = 6
    SmallWalk = 7
    Ascending = 8
    Descending = 9
    VariableWalk = 10

    def __str__(self):
        return self.name