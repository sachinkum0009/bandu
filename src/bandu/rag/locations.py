"""
Script to manage tools related to location
"""

from typing import Type

from pydantic import BaseModel, Field
from rai.tools.ros2.base import BaseTool

from bandu.rag.rag_client import RagChroma


class GetLocationToolInput(BaseModel):
    """Tool Input schema for getting location"""

    location_name: str = Field(
        ..., description="The name of location of which find the location"
    )


class GetLocationTool(BaseTool):
    """Tool for getting the list of locations known to the robot"""

    name: str = "get_locations"
    description: str = """
    This tool return the known location from already known locations, e.g. Kitchen, Bedroom, Living room, Dinning room, Play area, Garage, etc.
    It gives the result containing the 2D Coordinates x and y. These coordinates can be used by robot to navigate to the location using Nav2 Toolkit
    """
    args_schema: Type[GetLocationToolInput] = GetLocationToolInput

    def _run(self, location_name: str) -> str:
        """Returns the position of known location."""
        rag_chroma = RagChroma("locations")
        return rag_chroma.run_query(location_name)
