"""
Basic Agent to communicate with ros2 nodes

author: Sachin Kumar
date: 2025-09-15
"""

import os
from typing import List

from langchain_core.runnables import RunnableConfig
from langchain_core.tools import BaseTool
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
from rai.tools.ros2 import Nav2Toolkit
from rai_whoami.models import EmbodimentInfo
import rclpy
from agents.tools import GetRobotTemperatureTool, TellMeAJokeTool, GetROS2ImageTool
from rai.tools.time import WaitForSecondsTool

def create_agent(connector: ROS2Connector):
    """
    Create a navigation agent with specific tools and embodiment info.
    """
    tools: List[BaseTool] = [
        *Nav2Toolkit(connector=connector).get_tools(),
    ]

    llm = get_llm_model(model_type="complex_model", streaming=True)
    embodiment_info = EmbodimentInfo.from_file(
        "embodiments/navigation_embodiment.json"
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
