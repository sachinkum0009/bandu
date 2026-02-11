"""
App to integrate ROS2 with AI Agents using RAI

Author: Sachin Kumar
Date: 2025-08-12
"""

import rclpy

import chainlit as cl
from dotenv import load_dotenv
import logging
from langgraph.checkpoint.memory import InMemorySaver
from langchain_core.messages import HumanMessage
import os
from rai import get_llm_model, get_tracing_callbacks
from rai.communication.ros2 import ROS2Connector
from typing import List


from bandu.agents import AgentType, make_team, create_agent_node
from bandu.app import ToolTrackingCallback

load_dotenv()
ENABLE_AUTH = os.getenv("ENABLE_AUTH", "false").lower() == "true"

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(name="bandu")

## Initialize ROS2
rclpy.init()
connector = ROS2Connector(executor_type="single_threaded")
node = connector.node
node.declare_parameter("conversion_ratio", 1.0)

## Create the agents
basic = create_agent_node("basic", AgentType.BASIC, connector)
navigator = create_agent_node("navigator", AgentType.NAVIGATION, connector)
manipulator = create_agent_node("manipulator", AgentType.MANIPULATION, connector)
perception = create_agent_node("perception", AgentType.PERCEPTION, connector)

builder = make_team([basic, navigator, manipulator, perception])  # type: ignore

checkpointer = InMemorySaver()  # use sqlite in future
graph = builder.compile(checkpointer=checkpointer)

## summarizer
summarizer_llm = get_llm_model(model_type="simple_model", streaming=True)


if ENABLE_AUTH:

    @cl.password_auth_callback
    async def auth_callback(username: str, password: str):
        # Fetch the user matching username from your database
        # and compare the hashed password with the value stored in the database
        if (username, password) == ("admin", "admin"):
            return cl.User(
                identifier="admin",
                metadata={"role": "admin", "provider": "credentials"},
            )
        elif (username, password) == ("sachinkum0009", "123456"):
            return cl.User(
                identifier="user", metadata={"role": "user", "provider": "credentials"}
            )
        else:
            return None


@cl.set_starters
async def set_starters(user=None):
    return [
        cl.Starter(
            label="Morning routine ideation",
            message="Can you help me create a personalized morning routine that would help increase my productivity throughout the day? Start by asking me about my current habits and what activities energize me in the morning.",
            icon="/public/idea.png",
        ),
        cl.Starter(
            label="What is Robot's Temperature?",
            message="Can you help me understand the current temperature of the robot?",
            icon="/public/thermometer.png",
        ),
        cl.Starter(
            label="What does the robot see?",
            message="Please get an image from a ros2 topic '/image_raw'",
            icon="/public/camera.png",
        ),
    ]


@cl.on_message
async def on_message(message: cl.Message):
    # Get the unique session ID from Chainlit
    session_id = cl.user_session.get("id")
    logger.info(f"Session id: {session_id}")
    logger.info(f"Received message: {message.content}")
    # Create tool tracking callback
    tool_tracker = ToolTrackingCallback()

    config = {
        "callbacks": [*get_tracing_callbacks(), tool_tracker],
        "configurable": {"thread_id": session_id},
    }

    msg = cl.Message(content="", author="Agent")
    await msg.update()

    tool_name = None
    chunks = []
    agent_responses: List[str] = []

    # Create parent supervisor step
    async with cl.Step(name="Supervisor", type="llm") as supervisor_step:
        supervisor_step.input = message.content

        # Stream the graph with just the current user message
        # The checkpointer will handle conversation history per thread_id
        for chunk in graph.stream(
            {"messages": [HumanMessage(content=message.content)]},
            config=config,
        ):
            logger.info(f"Chunk: {chunk}")
            chunks.append(chunk)
            agent_response = next(iter(chunk.items()))[1]  # .get("messages", [""])

            # Check tool tracker for new tool calls
            if tool_tracker.tool_calls:
                for idx, tool_call in enumerate(tool_tracker.tool_calls):
                    tool_name = tool_call.get("name", "Unknown Tool")
                    tool_input = tool_call.get("input", "")
                    async with cl.Step(
                        name=f"🔧 {tool_name}", type="tool"
                    ) as tool_step:
                        tool_step.input = tool_input
                        # Check if we have a corresponding result
                        if idx < len(tool_tracker.tool_results):
                            tool_step.output = tool_tracker.tool_results[idx].get(
                                "output", ""
                            )
                        else:
                            tool_step.output = "Executing..."

                # Clear the tracked calls after creating steps
                tool_tracker.tool_calls = []
                tool_tracker.tool_results = []

            # Check for tool calls in ALL messages (including history)
            if "messages" in agent_response:
                all_messages = (
                    agent_response["messages"]
                    if isinstance(agent_response["messages"], list)
                    else [agent_response["messages"]]
                )

                for msg_item in all_messages:
                    # Check if message has tool_calls (AIMessage calling a tool)
                    if hasattr(msg_item, "tool_calls") and msg_item.tool_calls:
                        for tool_call in msg_item.tool_calls:
                            tool_name = tool_call.get("name", "Unknown Tool")
                            tool_args = tool_call.get("args", {})
                            logger.info(
                                f"Found tool call: {tool_name} with args: {tool_args}"
                            )
                            async with cl.Step(
                                name=f"🔧 {tool_name}", type="tool"
                            ) as tool_step:
                                tool_step.input = (
                                    str(tool_args) if tool_args else "No arguments"
                                )
                                tool_step.output = f"Executing tool: {tool_name}"

                    # Check if message is a ToolMessage (tool result)
                    if msg_item.__class__.__name__ == "ToolMessage":
                        tool_name = getattr(msg_item, "name", "Unknown Tool")
                        tool_content = getattr(msg_item, "content", "")
                        logger.info(f"Found tool result: {tool_name} = {tool_content}")
                        async with cl.Step(
                            name=f"✓ {tool_name} result", type="tool"
                        ) as tool_step:
                            tool_step.output = str(tool_content)

            # Call the appropriate agent function based on 'next' value
            if "next" in agent_response:
                next_agent = agent_response["next"]
                if next_agent == "basic":
                    async with cl.Step(name="Basic Agent", type="llm") as step:
                        step.output = "Processing with basic agent"
                elif next_agent == "navigator":
                    async with cl.Step(name="Navigator Agent", type="llm") as step:
                        step.output = "Processing with navigator agent"
                elif next_agent == "manipulator":
                    async with cl.Step(name="Manipulator Agent", type="llm") as step:
                        step.output = "Processing with manipulator agent"

            # Extract content from the agent response
            if "messages" in agent_response and len(agent_response["messages"]) > 0:
                message_content = agent_response["messages"][-1].content
                logger.info(f"Message Content: {message_content}")
                agent_responses.append(message_content)

                # Update the step output with the actual message content if we just created a step
                if "next" in agent_response:
                    async with cl.Step(
                        name=f"{agent_response['next'].title()} Agent Response",
                        type="llm",
                    ) as step:
                        step.output = message_content

        supervisor_step.output = f"Coordinated {len(agent_responses)} agent responses"

        # Get conversation history from checkpointer for summarizer context
        checkpoint_config = {"configurable": {"thread_id": session_id}}
        checkpoint_state = checkpointer.get(checkpoint_config)

        # Build conversation history context
        history_context = ""
        if checkpoint_state and "channel_values" in checkpoint_state:
            history_messages = checkpoint_state["channel_values"].get("messages", [])
            if history_messages:
                history_context = "\n\nConversation history:\n"
                for hist_msg in history_messages[-6:]:  # Last 6 messages (3 exchanges)
                    if hasattr(hist_msg, "content"):
                        msg_type = (
                            "User"
                            if hist_msg.__class__.__name__ == "HumanMessage"
                            else "Assistant"
                        )
                        history_context += f"{msg_type}: {hist_msg.content}\n"

        # Summarizer as a child step
        print(f"summarizing {len(agent_responses)} agent responses")
        prompt = "please read the following responses and provide a brief response to the user. "
        summary_prompt = (
            prompt
            + history_context
            + "\n\nCurrent query: "
            + message.content
            + "\n\nAgent responses: "
            + " ".join(agent_responses)
        )

        async with cl.Step(name="Summarizer", type="llm") as summarizer_step:
            summarizer_step.input = summary_prompt
            summary_content = ""

            # Stream the summarized response
            async for chunk in summarizer_llm.astream(summary_prompt):
                if hasattr(chunk, "content") and chunk.content:
                    content = (
                        chunk.content
                        if isinstance(chunk.content, str)
                        else str(chunk.content)
                    )
                    await msg.stream_token(content)
                    summary_content += content

            summarizer_step.output = summary_content

    print(f"Final summarized response: {msg.content}")
    print("-" * 100)
    await msg.update()
