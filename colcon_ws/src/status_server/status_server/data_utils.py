"""
This module provides utility functions for data manipulation and logging
as a class-based structure.

Class:
    DataUtils: Provides methods to convert dictionaries and strings to dataclass
               instances and extract point coordinates from strings.

Modules:
    os: Provides a way of using operating system-dependent functionality.
    traceback: Provides utilities for extracting, formatting, and printing stack traces.
    typing: Provides support for type hints.
    rclpy.impl.rcutils_logger: Provides logging functionality for ROS 2.

Attributes:
    logger (RcutilsLogger): Logger instance for logging errors and messages.
"""
import os
import traceback
from dataclasses import is_dataclass
from typing import TypeVar

from rclpy.impl.rcutils_logger import RcutilsLogger

T = TypeVar('T')


class DataUtils:
    """Utility class for data conversion and logging."""

    logger = RcutilsLogger(os.path.basename(__file__))

    @staticmethod
    def from_dict(input_class: type[T], data: dict) -> T:
        """Recursively converts a dictionary into an instance of the given dataclass."""
        try:
            if not is_dataclass(input_class):
                raise ValueError(f"{input_class} must be a dataclass")

            init_data = {}
            # Field Types
            field_types = {
                field.name: field.type for field in input_class.__dataclass_fields__.values()}

            for field, field_type in field_types.items():
                value = data[field]
                if is_dataclass(field_type) and isinstance(field_type, type):
                    value = DataUtils.from_dict(field_type, value)
                init_data[field] = value

            return input_class(**init_data)
        except Exception as e:
            DataUtils.logger.error(
                f"Error in from_dict: {e}\n{traceback.format_exc()}")
            raise

    @staticmethod
    def str_to_model(s: str, data_class: type[T]) -> T:
        """
        Convert a comma-separated string into an instance of a given class.

        Example:
            >>> @dataclass
            ... class Point:
            ...     x: float
            ...     y: float
            ...     z: float
            >>> DataUtils.str_to_model("(1.0, 2.0, 3.0)", Point)
            Point(x=1.0, y=2.0, z=3.0)
        """
        parts = s.strip("()").split(",")
        parts = [float(p.strip()) for p in parts if p.strip()]
        return data_class(*parts[:len(data_class.__annotations__)])

    @staticmethod
    def get_point_coordinates(point_string: str) -> tuple[float, float] | None:
        """Extracts x and y coordinates from a PostgreSQL point string."""
        try:
            point_string = point_string.strip('()')
            x_str, y_str = point_string.split(',')
            return (float(x_str), float(y_str))
        except (ValueError, AttributeError) as e:
            DataUtils.logger.error(
                f"Error in get_point_coordinates: {e}\n{traceback.format_exc()}")
            return None
