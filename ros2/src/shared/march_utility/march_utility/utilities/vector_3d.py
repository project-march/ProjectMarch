"""This module contains a basic Vector3d class."""
from __future__ import annotations
from math import sqrt
from typing import Iterator

from march_utility.exceptions.general_exceptions import IncorrectCoordinateError


class Vector3d:
    """A 3d vector class, used for specifying positions and directional velocities.

    Args:
        x (float): x_coordinate.
        y (float): y_coordinate.
        z (float): z_coordinate.
    """

    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other: Vector3d) -> Vector3d:
        """Add two vectors together.

        Args:
            other (Vector3d): The vector to add.

        Returns:
            Vector3d. The new vector.
        """
        x = self.x + other.x
        y = self.y + other.y
        z = self.z + other.z
        return Vector3d(x, y, z)

    def __sub__(self, other: Vector3d) -> Vector3d:
        """Subtract two vectors.

        Args:
            other (Vector3d): The vector to subtract.

        Returns:
            Vector3d. The new vector.
        """
        x = self.x - other.x
        y = self.y - other.y
        z = self.z - other.z
        return Vector3d(x, y, z)

    def __truediv__(self, factor: float) -> Vector3d:
        """Divide vector with a certain factor.

        Args:
            factor (float): The factor to use.

        Returns:
            Vector3d. The new vector.
        """
        x = self.x / factor
        y = self.y / factor
        z = self.z / factor
        return Vector3d(x, y, z)

    def __mul__(self, factor: float) -> Vector3d:
        """Multiply vector with a certain factor.

        Args:
            factor (float): The factor to use.

        Returns:
            Vector3d. The new vector.
        """
        x = self.x * factor
        y = self.y * factor
        z = self.z * factor
        return Vector3d(x, y, z)

    def __getitem__(self, direction: str) -> float:
        """Get one direction of the vector.

        Args:
            direction (str): The direction to get, either 'x', 'y', or 'z'.

        Returns:
            float. The coordinate value of the given direction.
        """
        return self.as_dictionary()[direction]

    def __iter__(self) -> Iterator:
        """Get vector as iterator."""
        yield from [self.x, self.y, self.z]

    def __eq__(self, other: object) -> bool:
        """Check if two vectors are equal.

        Args:
            other (object): The vector to compare to.

        Returns:
            bool. Whether the vectors are equal.
                `True` if it is an instance of Vector3d with same values for x, y and z.
                `False`, otherwise.
        """
        if not isinstance(other, Vector3d):
            return False
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __repr__(self) -> str:
        """Represent the vector in a string.

        ReturnsL
            str. A string with the vector coordinates.
        """
        return f"x: {self.x}, y: {self.y}, z: {self.z}"

    @classmethod
    def from_dictionary(cls, dic: dict) -> Vector3d:
        """Create a Vector3d object from a dictionary with keys 'x', 'y' and 'z'."""
        if {"x", "y", "z"} != set(dic.keys()):
            raise IncorrectCoordinateError()
        return cls(dic["x"], dic["y"], dic["z"])

    def as_dictionary(self) -> dict:
        """Get the vector as a dictionary."""
        return {"x": self.x, "y": self.y, "z": self.z}

    def norm(self) -> float:
        """Get the normalized vector."""
        return sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    @staticmethod
    def is_close_enough(vector1: Vector3d, vector2: Vector3d, tolerance: float = 0.0001) -> bool:
        """Check whether the normalized vectors are within a given tolerance."""
        return (vector1 - vector2).norm() <= tolerance

    @staticmethod
    def size() -> int:
        """Get the size of the vector."""
        return 3
