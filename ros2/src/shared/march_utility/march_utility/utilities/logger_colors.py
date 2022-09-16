"""This class is made so that one can give colored output in the terminal."""
from enum import Enum


class TerminalColors(Enum):
    """Class with constants to add in logger message to color your terminal output.

    Example:
        logger.info(f"The word {TerminalColors.BLUE}'red'{TerminalColors.END} has a blue color.")
    """

    HEADER = "\033[95m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    GREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"
    END = "\033[0m"
