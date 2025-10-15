from typing import Literal, Type

from pydantic import BaseModel, Field
from rai.communication.ros2 import ROS2Message
from rai.messages import MultimodalArtifact, preprocess_image
from rai.tools.ros2.base import BaseROS2Tool, BaseTool

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

from typing import Tuple

# Tool input schema for temperature (no input needed)
class GetRobotTemperatureToolInput(BaseModel):
    pass


# Tool for getting robot temperature
class GetRobotTemperatureTool(BaseROS2Tool):
    """Tool for getting the robot temperature."""
    name: str = "get_robot_temperature"
    description: str = "Get the robot temperature"
    args_schema: Type[GetRobotTemperatureToolInput] = GetRobotTemperatureToolInput  # type: ignore

    def _run(self) -> str:
        """Return the robot temperature as a constant value."""
        print("temperature tool is called")
        return "The robot temperature is 55°C"

class TellMeAJokeInput(BaseModel):
    """
    Input schema for the TellMeAJokeTool.

    This schema defines the expected input parameters for the tool.
    The Field class provides additional metadata that helps the LLM understand the parameters's purpose and constraints.
    Fields must include a type annotation.
    """
    topic_name: str = Field(description="The name of the topic to tell joke about")

class TellMeAJokeTool(BaseROS2Tool):
    """Tool for telling a joke about a specific topic.

    The following fields are crucial for LLM understanding:

    name: A unique identifier that the LLM uses to reference this tool
    description: A natural language explanation that helps the LLM understand when to use this tool
    args_schema: Links to the input schema, helping the LLM understand required parameters
    """
    name: str = "tell_me_a_joke"
    description: str = "Tells a joke about a specific topic"
    args_schema: Type[TellMeAJokeInput] = TellMeAJokeInput # type: ignore

    def _run(self, topic_name: str) -> str:
        """Return a joke about the specified topic."""
        jokes = {
            "robot": "Why was the robot so bad at soccer? Because it kept kicking up sparks!",
            "cat": "Why don't cats play poker in the jungle? Too many cheetahs!",
            "dog": "Why did the dog sit in the shade? Because he didn't want to be a hot dog!",
            "computer": "Why was the computer cold? It left its Windows open!",
            "math": "Why was the equal sign so humble? Because it knew it wasn't less than or greater than anyone else!"
        }
        return jokes.get(topic_name.lower(), f"Sorry, I don't have a joke about {topic_name}.")

class GetROS2ImageToolInput(BaseModel):
    topic: str = Field(..., description="The topic to receive the image from")
    timeout_sec: float = Field(1.0, description="The timeout in seconds")

class GetROS2ImageTool(BaseROS2Tool):
    # connector: ROS2Connector
    name: str = "get_ros2_image"
    description: str = "Get an image from a ROS2 topic"
    args_schema: Type[GetROS2ImageToolInput] = GetROS2ImageToolInput # type: ignore
    response_format: Literal["content", "content_and_artifact"] = "content"

    def _run(
        self, topic: str, timeout_sec: float = 1.0
    ) -> str:
        if not self.is_readable(topic):
            raise ValueError(f"Topic {topic} is not readable")
        message = self.connector.receive_message(topic, timeout_sec=timeout_sec)
        msg_type = type(message.payload)
        if msg_type == Image:
            image = CvBridge().imgmsg_to_cv2(  # type: ignore
                message.payload, desired_encoding="bgr8"
            )
        elif msg_type == CompressedImage:
            image = CvBridge().compressed_imgmsg_to_cv2(  # type: ignore
                message.payload, desired_encoding="bgr8"
            )
        else:
            raise ValueError(
                f"Unsupported message type: {message.metadata['msg_type']}"
            )
        # print("processing the image")
        return preprocess_image(image)