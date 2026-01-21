"""
Basic Agent to communicate with ros2 nodes

author: Sachin Kumar
date: 2025-08-12
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
from rai.messages.multimodal import SystemMultimodalMessage, AIMessage, HumanMultimodalMessage
from rai.tools.ros2 import ROS2Toolkit
from rai_whoami.models import EmbodimentInfo
import rclpy

from agents.mcp_adapter import get_mcp_adapter_tools
from agents.tools import GetRobotTemperatureTool, TellMeAJokeTool, GetROS2ImageTool

async def create_agent(connector: ROS2Connector):
    tools: List[BaseTool] = [
        GetRobotTemperatureTool(connector=connector),
        TellMeAJokeTool(connector=connector),
        *ROS2Toolkit(connector=connector).get_tools(),
        GetROS2ImageTool(connector=connector),
    ]
    mcp_tools = await get_mcp_adapter_tools()

    llm = get_llm_model(model_type="complex_model", streaming=True)
    embodiment_info = EmbodimentInfo.from_file(
        "embodiments/basic_embodiment.json"
    )
    agent = create_conversational_agent(
        llm=llm,
        tools=tools+mcp_tools, # add ros2 tools and mcp tools
        system_prompt=embodiment_info.to_langchain(),
    )
    return agent
