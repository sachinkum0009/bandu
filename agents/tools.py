from typing import Literal, Type

from pydantic import BaseModel, Field
from rai.communication.ros2 import ROS2Message
from rai.tools.ros2.base import BaseROS2Tool

# Tool input schema for temperature (no input needed)
class GetRobotTemperatureToolInput(BaseModel):
    pass


# Tool for getting robot temperature
class GetRobotTemperatureTool(BaseROS2Tool):
    """Tool for getting the robot temperature (returns constant 20°C)."""
    name: str = "get_robot_temperature"
    description: str = "Get the robot temperature (returns constant 20°C)"
    args_schema: Type[GetRobotTemperatureToolInput] = GetRobotTemperatureToolInput

    def _run(self) -> str:
        """Return the robot temperature as a constant value."""
        print("temperature tool is called")
        return "The robot temperature is 20°C"