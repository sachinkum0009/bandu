"""
Megamind Agent to manage multiple agents

author: Sachin Kumar
date: 2025-09-07
"""
import rclpy

from rai.agents.langchain.core.megamind import (
    MegamindState,
    create_megamind,
    Executor,
    get_initial_megamind_state,
)
from rai import get_llm_model, get_tracing_callbacks
from rai.communication.ros2 import (
    ROS2Connector,
    ROS2HRIConnector,
    ROS2HRIMessage,
    ROS2Message,
)
from rai.messages import HumanMultimodalMessage
from rai.tools.ros2 import ROS2Toolkit
from langchain_core.tools import BaseTool

from agents.tools import GetRobotTemperatureTool, TellMeAJokeTool, GetROS2ImageTool
from typing import List

from megamind_response import MegamindResponse
chunks = []

def test(connector: ROS2Connector):
    """
    Test Megamind agent with multiple executors
    """
    tools: List[BaseTool] = [
        GetRobotTemperatureTool(connector=connector),
        TellMeAJokeTool(connector=connector),
    ]
    llm = get_llm_model(model_type="simple_model", streaming=True)
    basic_executor = Executor(
        name="basic_executor",
        llm=llm,
        tools=tools,
        system_prompt="You are a helpful assistant that has tools like get_robot_temperature to get the current temperature of the robot.",
    )
    navigation_executor = Executor(
        name="navigation_executor",
        llm=llm,
        tools=[
            *ROS2Toolkit(connector=connector).get_tools(),
        ],
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
    initial_message = get_initial_megamind_state(
        task="please call basic_executor to get current temperature of robot"
    )
    for chunk in graph.stream(initial_message, config={"callbacks": get_tracing_callbacks()}):
        # print(f"type(chunk): {type(chunk)}")
        print(chunk)
        print("*" * 100)
        chunks.append(chunk)

    # megamind_response = MegamindResponse.from_dict(chunks[-1])
    # print("Final Response from MegamindResponse:", megamind_response.final_response)
    # print("Test completed.")

    response = chunks[-1]
    final_response = response["megamind"]["messages"][-1].content
    print(f"Final Response from chunks: {final_response}")

if __name__ == "__main__":
    rclpy.init()
    connector = ROS2Connector(executor_type="single_threaded")
    test(connector)
