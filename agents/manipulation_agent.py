"""
Manipulation Agent to communicate with ros2 nodes

author: Sachin Kumar
date: 2025-09-17
"""

import os
from typing import List

from langchain_core.runnables import RunnableConfig
from langchain_core.tools import BaseTool
from rai.tools.ros2.base import BaseROS2Tool
from rai import get_llm_model
from rai.agents import BaseAgent
from rai.agents.langchain import (
    HRICallbackHandler,
    ReActAgentState,
    create_react_runnable,
    ReActAgent
)
from rai.agents.langchain.core import create_conversational_agent
from rai.communication.ros2 import (
    ROS2Connector,
    ROS2HRIConnector,
    ROS2HRIMessage,
    ROS2Message,
)
from rai.messages.multimodal import SystemMultimodalMessage
from rai.tools.ros2 import ROS2Toolkit
from rai.tools.ros2.manipulation import (
    MoveToPointTool,
    GetObjectPositionsTool,
    MoveObjectFromToTool,
    ResetArmTool,
)
from rai_whoami.models import EmbodimentInfo
import rclpy
from agents.tools import GetRobotTemperatureTool, TellMeAJokeTool, GetROS2ImageTool
from rai.tools.ros2.simple import GetROS2ImageConfiguredTool
# from rai_open_set_vision.tools import GetGrabbingPointTool

def create_agent(connector: ROS2Connector):
    """
    Create a manipulation agent with specific tools and embodiment info.
    """
    tools: List[BaseROS2Tool] = [
        MoveToPointTool(connector=connector, manipulator_frame="base_link"),
        # GetObjectPositionsTool(
        #     connector=connector,
        #     target_frame="panda_link0",
        #     source_frame="camera_link",
        #     camera_topic="/image_raw",
        #     depth_topic="/depth_image_raw",
        #     camera_info_topic="/color_camera_info",
        #     get_grabbing_point_tool=GetGrabbingPointTool(connector=connector),
        # ),
        MoveObjectFromToTool(connector=connector, manipulator_frame="panda_link0"),
        ResetArmTool(connector=connector, manipulator_frame="panda_link0"),
        GetROS2ImageConfiguredTool(connector=connector, topic="/image_raw"),
    ]

    llm = get_llm_model(model_type="complex_model", streaming=True)
    embodiment_info = EmbodimentInfo.from_file(
        "embodiments/manipulation_embodiment.json"
    )
    agent = create_conversational_agent(
        llm=llm,
        tools=tools,
        system_prompt=embodiment_info.to_langchain(),
    )
    # agent = create_react_runnable(
    #     llm=llm,
    #     tools=tools,
    #     system_prompt=embodiment_info.to_langchain(),
    # )
    return agent
