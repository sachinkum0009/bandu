# MIT License

# Copyright (c) 2025 Sachin Kumar

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


"""
Manipulation Agent to communicate with ros2 nodes

author: Sachin Kumar
date: 2025-09-17
"""

from typing import List

from rai.tools.ros2.base import BaseROS2Tool
from rai import get_llm_model
from rai.agents.langchain.core import create_react_runnable
from rai.communication.ros2 import (
    ROS2Connector,
)
from rai.tools.ros2.manipulation import (
    MoveToPointTool,
    MoveObjectFromToTool,
    ResetArmTool,
)
from rai_whoami.models import EmbodimentInfo

# from rai_open_set_vision.tools import GetGrabbingPointTool


def create_agent(connector: ROS2Connector, manipulator_frame: str = "base_link"):
    """
    Create a manipulation agent with specific tools and embodiment info.
    """
    tools: List[BaseROS2Tool] = [
        MoveToPointTool(connector=connector, manipulator_frame=manipulator_frame),
        # GetObjectPositionsTool(
        #     connector=connector,
        #     target_frame=manipulator_frame,
        #     source_frame="camera_link",
        #     camera_topic="/image_raw",
        #     depth_topic="/depth_image_raw",
        #     camera_info_topic="/color_camera_info",
        #     get_grabbing_point_tool=GetGrabbingPointTool(connector=connector),
        # ),
        MoveObjectFromToTool(connector=connector, manipulator_frame=manipulator_frame),
        ResetArmTool(connector=connector, manipulator_frame=manipulator_frame),
        # GetROS2ImageConfiguredTool(connector=connector, topic="/image_raw"), # NOTE: Should be moved to perception agent
    ]

    llm = get_llm_model(model_type="complex_model", streaming=True)
    embodiment_info = EmbodimentInfo.from_file(
        "embodiments/manipulation_embodiment.json"
    )
    # agent = create_conversational_agent(
    #     llm=llm,
    #     tools=tools,
    #     system_prompt=embodiment_info.to_langchain(),
    # )
    agent = create_react_runnable(
        llm=llm,
        tools=tools,
        system_prompt=embodiment_info.to_langchain(),
    )
    return agent
