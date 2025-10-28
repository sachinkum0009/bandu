"""
Basic Agent to communicate with ros2 nodes
author: Sachin Kumar
date: 2025-09-07
"""

from langgraph.graph.state import CompiledStateGraph
from langgraph.graph import StateGraph, MessagesState, START, END
from langgraph.types import Command
from langgraph.prebuilt import create_react_agent
from langchain_core.language_models.chat_models import BaseChatModel
from langchain_core.messages import HumanMessage, trim_messages
from langchain_ollama import ChatOllama
from typing import List, Tuple, TypedDict, Literal
from rai.communication.ros2 import (
    ROS2Connector,
    ROS2HRIConnector,
    ROS2HRIMessage,
    ROS2Message,
)
from functools import partial
from agents import AgentType, make_team, create_agent_node, State

import rclpy

def main():
    rclpy.init()
    connector = ROS2Connector(executor_type="single_threaded")
    node = connector.node
    node.declare_parameter("conversion_ratio", 1.0)

    # Create agents using the new function
    agent1 = create_agent_node("agent1", AgentType.BASIC, connector)
    agent2 = create_agent_node("influencer", AgentType.INFLUENCER, connector)
    agent3 = create_agent_node("agent3", AgentType.BASIC, connector)

    builder = make_team([agent1, agent2, agent3])

    graph = builder.compile()
    # graph.get_graph().draw_mermaid_png(output_file_path="team_graph7.png")
    graph.get_graph().print_ascii()
    # graph.get_graph().draw_png(output_file_path="team_graph9.png")

    print("Starting conversation...")


    for s in graph.stream(
        # {"messages": [("user", "What is the temperature of the robot?")]},
        {"messages": [("user", "Who is Taylor Swift?")]},
        {"recursion_limit": 5},
    ):
        print(s)
        print("---")



    # graph.invoke()
    print("Conversation ended.")


if __name__ == "__main__":
    main()
