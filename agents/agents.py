"""
Basic Agent to communicate with ros2 nodes

author: Sachin Kumar
date: 2025-09-07
"""

from enum import Enum
from typing import List, Optional, Literal, TypedDict, Tuple, Callable
from rai import get_llm_model
from rai.communication.ros2 import (
    ROS2Connector,
    ROS2HRIConnector,
    ROS2HRIMessage,
    ROS2Message,
)

from langgraph.types import Command
from langgraph.graph import StateGraph, MessagesState, START, END
from langgraph.graph.state import CompiledStateGraph
from langchain_core.language_models.chat_models import BaseChatModel
from langchain_core.messages import HumanMessage, trim_messages

class State(MessagesState):
    next: str

class AgentType(str, Enum):
    BASIC = "basic"
    SEARCH = "search"
    WEB_SCRAPER = "web_scraper"
    SUPERVISOR = "supervisor"
    INFLUENCER = "influencer"
    NAVIGATION = "navigation"
    PERCEPTION = "perception"
    MANIPULATION = "manipulation"

def make_summarizer_node(llm: BaseChatModel) -> Callable[[State], Command[Literal["supervisor"]]]:
    system_prompt = (
        "You are a summarizer tasked with summarizing the conversation between the user and multiple agents."
        " Summarize the conversation in a concise manner."
    )
    
    def summarizer_node(state: State) -> Command[Literal["supervisor"]]:
        """An LLM-based summarizer."""
        messages = [
            {"role": "system", "content": system_prompt},
        ] + state["messages"]
        trimmed_messages = trim_messages(messages, max_tokens=3000, llm=llm)
        response = llm.invoke(trimmed_messages)
        return Command(
            update={
                "messages": [
                    HumanMessage(content=response.content, name="summarizer")
                ]
            },
            goto="supervisor",
        )
    
    return summarizer_node


def make_supervisor_node(llm: BaseChatModel, members: list[str]) -> Callable[[State], Command]:
    options = ["FINISH"] + members
    system_prompt = (
        "You are a supervisor tasked with help the user by selecting appropriate agent which will be used to answer the user query in a conversation"
        f" use these workers: {members}. to get the answer and then reply with FINISH."
        "Basic agent can be used to get temperature of robot, tell jokes and also use ros2 tools for publishing and subscribing to topics, calling services and actions."
        "Navigator agent can be used to navigate the robot to a location."
        "Manipulator agent can be used to control the robot's arms and grippers."
        "Perception agent can be used to get images from the robot's camera."
        "Summarize the response from the workers and provide a final answer."
        # " respond with the worker to act next. If the task is complete and end the conversation"
        # " task and respond with their results and status. When finished,"
        # " respond with FINISH."
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

def create_node(state: State, agent: CompiledStateGraph, agent_name: str) -> Command[Literal["supervisor"]]:
    """Create a node function that invokes an agent and returns to supervisor."""
    result = agent.invoke(state)
    return Command(
        update={
            "messages": [
                HumanMessage(content=result["messages"][-1].content, name=agent_name)
            ]
        },
        # We want our workers to ALWAYS "report back" to the supervisor when done
        goto="supervisor",
    )

def make_team(members: List[Tuple[str, CompiledStateGraph]]):
    llm = get_llm_model(model_type="complex_model", streaming=True)
    sub_agents = [member[0] for member in members]
    supervisor_node = make_supervisor_node(llm, sub_agents)

    # summarizer_node = make_summarizer_node(llm)
    
    graph = StateGraph(State)
    graph.add_node("supervisor", supervisor_node)
    
    for member in members:
        print(f"Adding agent: {member[0]}")
        graph.add_node(member[0], member[1])
    
    graph.add_edge(START, "supervisor")
    return graph

def create_agent_node(name: str, agent_type: AgentType, connector: ROS2Connector) -> Tuple[str, callable]:
    """Create an agent and return a tuple of (name, agent)."""
    if agent_type == AgentType.BASIC:
        from agents.basic_agent import create_agent as create_basic_agent
        agent = create_basic_agent(connector)
        
        def node(state: State) -> Command[Literal["supervisor"]]:
            return create_node(state, agent, name)
        
        return (name, node)
        # return (name, agent)
    elif agent_type == AgentType.INFLUENCER:
        from agents.influencer_agent import create_agent as create_influencer_agent
        agent = create_influencer_agent(connector)

        def node(state: State) -> Command[Literal["supervisor"]]:
            return create_node(state, agent, name)
        
        return (name, node)
        
        # def node(state: State) -> Command[Literal["supervisor"]]:
        #     return create_node(state, agent, name)
        
        # return (name, node)
        # return (name, agent)
    elif agent_type == AgentType.NAVIGATION:
        from agents.navigation_agent import create_agent as create_navigation_agent
        agent = create_navigation_agent(connector)
        def node(state: State) -> Command[Literal["supervisor"]]:
            return create_node(state, agent, name)
        
        return (name, node)
        
        # return (name, agent)
    
    elif agent_type == AgentType.MANIPULATION:
        from agents.manipulation_agent import create_agent as create_manipulation_agent
        agent = create_manipulation_agent(connector)

        def node(state: State) -> Command[Literal["supervisor"]]:
            return create_node(state, agent, name)

        return (name, node)
    
    elif agent_type == AgentType.PERCEPTION:
        from agents.perception_agent import create_agent as create_perception_agent
        agent = create_perception_agent(connector)
        def node(state: State) -> Command[Literal["supervisor"]]:
            return create_node(state, agent, name)
        
        return (name, node)
        # return (name, agent)

    else:
        raise ValueError(f"Unknown agent type: {agent_type}")
