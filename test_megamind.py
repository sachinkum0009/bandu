"""
Megamind Agent to manage multiple agents

author: Sachin Kumar
date: 2025-09-07
"""

import rclpy

from rai.agents.langchain.core.megamind import MegamindState, create_megamind, Executor
from rai import get_llm_model
from rai.communication.ros2 import (
    ROS2Connector,
    ROS2HRIConnector,
    ROS2HRIMessage,
    ROS2Message,
)
from rai.messages import HumanMultimodalMessage

from langchain_core.tools import BaseTool

from agents.tools import GetRobotTemperatureTool, TellMeAJokeTool, GetROS2ImageTool
from typing import List


def test(connector: ROS2Connector):
    tools: List[BaseTool] = [
        GetRobotTemperatureTool(connector=connector),
        TellMeAJokeTool(connector=connector),
    ]
    llm = get_llm_model(model_type="simple_model", streaming=True)
    basic_executor = Executor(
        name="basic_executor",
        llm=llm,
        tools=tools,
        system_prompt="You are a helpful assistant that can perform basic tasks.",
    )
    navigation_executor = Executor(
        name="navigation_executor",
        llm=llm,
        tools=tools,
        system_prompt="You are a helpful assistant that can perform navigation tasks.",
    )
    manipulator_executor = Executor(
        name="manipulator_executor",
        llm=llm,
        tools=tools,
        system_prompt="You are a helpful assistant that can perform manipulation tasks.",
    )

    llm = get_llm_model(model_type="complex_model", streaming=True)
    graph = create_megamind(
        megamind_llm=llm,
        megamind_system_prompt=" You are Megamind, a master coordinator of multiple specialized agents. Your job is to delegate tasks to the appropriate agents based on their expertise and capabilities. You must analyze the user's requests, determine which agent is best suited to handle each part of the request, and coordinate the execution of these tasks to provide a comprehensive response back to the user. Always consider the strengths and limitations of each agent when making your decisions. ",
        executors=[basic_executor, navigation_executor, manipulator_executor],
        task_planning_prompt="Write a detailed plan to complete the task",
    )
    graph.get_graph().draw_mermaid_png(output_file_path="megamind_graph2.png")

    print("Starting conversation...")
    for chunk in graph.stream(
        {"messages": [HumanMultimodalMessage(content="please call basic_executor to get current temperature of robot and then call manipulator robot to list the ros2 topics")]},
    ):
        print(chunk)

    print("=" * 20)


if __name__ == "__main__":
    rclpy.init()
    connector = ROS2Connector(executor_type="single_threaded")
    test(connector)
