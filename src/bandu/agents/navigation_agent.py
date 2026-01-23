"""
Basic Agent to communicate with ros2 nodes

author: Sachin Kumar
date: 2025-09-15
"""

from typing import List

from langchain_core.tools import BaseTool
from rai import get_llm_model
from rai.agents.langchain.core import create_conversational_agent
from rai.communication.ros2 import (
    ROS2Connector,
)
from rai.tools.ros2 import Nav2Toolkit
from rai_whoami.models import EmbodimentInfo

from bandu.rag.locations import GetLocationTool


def create_agent(connector: ROS2Connector):
    """
    Create a navigation agent with specific tools and embodiment info.
    """
    tools: List[BaseTool] = [
        *Nav2Toolkit(connector=connector).get_tools(),
        GetLocationTool(),
    ]

    llm = get_llm_model(model_type="complex_model", streaming=True)
    embodiment_info = EmbodimentInfo.from_file("embodiments/navigation_embodiment.json")
    agent = create_conversational_agent(
        llm=llm,
        tools=tools,
        system_prompt=embodiment_info.to_langchain(),
    )
    return agent
