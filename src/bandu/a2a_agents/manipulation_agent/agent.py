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

from rai import get_llm_model
from rai_whoami.models import EmbodimentInfo
from rai.tools.ros2.base import BaseROS2Tool
from rai.tools.ros2.manipulation import (
    MoveToPointTool,
    MoveObjectFromToTool,
    ResetArmTool,
)
from rai.communication.ros2 import (
    ROS2Connector,
)
from rai.tools.ros2.simple import GetROS2ImageConfiguredTool
from rai.agents.langchain.core import create_react_runnable

# from langchain.messages import SystemMessage, HumanMessage


class ManipulatorAgent:
    def __init__(self, connector: ROS2Connector):
        llm = get_llm_model(model_type="complex_model", streaming=True)
        embodiment_info = EmbodimentInfo.from_file(
            "embodiments/manipulation_embodiment.json"
        )
        tools: list[BaseROS2Tool] = [
            MoveToPointTool(connector=connector, manipulator_frame="base_link"),
            MoveObjectFromToTool(connector=connector, manipulator_frame="panda_link0"),
            ResetArmTool(connector=connector, manipulator_frame="panda_link0"),
            GetROS2ImageConfiguredTool(connector=connector, topic="/image_raw"),
        ]

        self.agent = create_react_runnable(
            llm=llm,
            tools=tools,  # type: ignore
            system_prompt=embodiment_info.to_langchain(),
        )

    async def invoke(self, msg: str) -> str:
        res = await self.agent.ainvoke({"messages": msg})
        return res
