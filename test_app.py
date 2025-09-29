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

from agents import AgentType, create_agent, State

llm = ChatOllama(model="llama3.1", temperature=0.7)
agent = create_react_agent(llm, tools=[])

def make_supervisor_node(llm: BaseChatModel, members: list[str]) -> str:
    options = ["FINISH"] + members
    system_prompt = (
        "You are a supervisor tasked with managing a conversation between the"
        f" following workers: {members}. Given the following user request,"
        " respond with the worker to act next. Each worker will perform a"
        " task and respond with their results and status. When finished,"
        " respond with FINISH."
    )

    class Router(TypedDict):
        """Worker to route to next. If no workers needed, route to FINISH."""

        next: Literal[*options]

    def supervisor_node(state: State) -> Command[Literal[*members, "__end__"]]:
        """An LLM-based router."""
        messages = [
            {"role": "system", "content": system_prompt},
        ] + state["messages"]
        response = llm.with_structured_output(Router).invoke(messages)
        goto = response["next"]
        if goto == "FINISH":
            goto = END

        return Command(goto=goto, update={"next": goto})

    return supervisor_node

def create_node(state: State, agent: CompiledStateGraph) -> Command[Literal["supervisor"]]:
    
    result = agent.invoke(state)
    return Command(
        update={
            "messages": [
                HumanMessage(content=result["messages"][-1].content, name="agent1")
            ]
        },
        # We want our workers to ALWAYS "report back" to the supervisor when done
        goto="supervisor",
    )

def make_team(members: List[Tuple[str, CompiledStateGraph]]):
    llm = ChatOllama(model="llama3.1", temperature=0.7)
    sub_agents = [member[0] for member in members]
    supervisor_node = make_supervisor_node(llm, sub_agents)
    graph = StateGraph(State)
    graph.add_node("supervisor", supervisor_node)
    for member in members:
        print(f"Adding member: {member[0]}")
        graph.add_node(member[0], member[1])
        graph.add_edge(member[0], "supervisor")  # Always route agents back to supervisor
        # graph.add_node(member[0], partial(create_node, agent=agent))
    graph.add_edge(START, "supervisor")
    return graph

def main():
    connector = ROS2Connector(executor_type="single_threaded")
    node = connector.node
    node.declare_parameter("conversion_ratio", 1.0)

    agent1 = create_agent("agent1", AgentType.BASIC, connector)
    agent2 = create_agent("agent2", AgentType.BASIC, connector)
    agent3 = create_agent("agent3", AgentType.BASIC, connector)
    # agent2 = create_agent("agent2", AgentType.SEARCH)
    # agent3 = create_agent("agent3", AgentType.WEB_SCRAPER)

    builder = make_team([agent1, agent2, agent3])
    graph = builder.compile()
    graph.get_graph().draw_mermaid_png(output_file_path="team_graph4.png")
    # graph.get_graph().print_ascii()
    

if __name__ == "__main__":
    main()



