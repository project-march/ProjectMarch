"""This module contains a Enum for possible Sides to operate on."""
from enum import Enum


class Side(Enum):
    """Enum for selecting a side."""

    right = "right"
    left = "left"
    both = "both"
