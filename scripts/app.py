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
App to integrate ROS2 with AI Agents using RAI

Author: Sachin Kumar
Date: 2025-08-12
"""

try:
    import rclpy  # type: ignore
except ImportError:
    raise ImportError(
        "rclpy is not installed. Please install ROS2 and source workspace to run this application."
    )

import asyncio
import logging
import os
import time
from typing import List

import chainlit as cl
from dotenv import load_dotenv
from langchain_core.messages import HumanMessage
from langgraph.checkpoint.memory import InMemorySaver
from rai import get_tracing_callbacks
from rai.communication.ros2 import ROS2Connector

from bandu.agents import AgentType, create_agent_node, make_team
from bandu.app import ToolTrackingCallback
from bandu.bridge import ImageBridge
from bandu.logger.logger_config import get_logger, setup_logging

load_dotenv()
ENABLE_AUTH = os.getenv("ENABLE_AUTH", "false").lower() == "true"


setup_logging(logging.INFO)
logger = get_logger("bandu")

## Initialize ROS2
rclpy.init()
connector = ROS2Connector(executor_type="single_threaded")
node = connector.node
node.declare_parameter("conversion_ratio", 1.0)  # type: ignore

## a2a supervisor
# supervisor_agent = create_agent(connector)

## Create the agents
basic = create_agent_node("basic", AgentType.BASIC, connector)
navigator = create_agent_node("navigator", AgentType.NAVIGATION, connector)
manipulator = create_agent_node(
    "manipulator", AgentType.MANIPULATION, connector, manipulator_frame="panda_link0"
)
perception = create_agent_node("perception", AgentType.PERCEPTION, connector)

builder = make_team([basic, navigator, manipulator, perception])  # type: ignore

checkpointer = InMemorySaver()  # use sqlite in future
graph = builder.compile(checkpointer=checkpointer)


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
            message="Can you tell me what is the current temperature of the robot?",
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
    debug_start_time = time.time()
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

    image_bridge = ImageBridge()
    image_bridge.get_image("my_topic")
    img = image_bridge.get_image("topic")
    img

    tool_name = None
    chunks = []
    agent_responses: List[str] = []

    # Create parent supervisor step
    async with cl.Step(name="Supervisor", type="llm") as supervisor_step:
        supervisor_step.input = message.content

        # Show processing indicator
        processing_msg = cl.Message(
            content="🔄 Processing your request...", author="System"
        )
        await processing_msg.send()

        # Stream the graph with just the current user message
        # The checkpointer will handle conversation history per thread_id
        # Use asyncio to prevent blocking the event loop
        def stream_graph():
            """Run graph.stream in a non-blocking manner"""
            return list(
                graph.stream(
                    {"messages": [HumanMessage(content=message.content)]},
                    config=config,
                )
            )

        try:
            # Run the blocking stream call in a thread pool to prevent UI freezing
            stream_chunks = await asyncio.wait_for(
                asyncio.to_thread(stream_graph),
                timeout=5 * 60.0,  # 5 minute timeout # TODO: Add param in config file
            )

            # Remove processing indicator
            await processing_msg.remove()

        except asyncio.TimeoutError:
            await processing_msg.remove()
            error_msg = "⚠️ Request timed out after 2 minutes. The LLM is taking too long to respond. Please try a simpler query."
            await cl.Message(content=error_msg, author="System").send()
            return
        except Exception as e:
            await processing_msg.remove()
            error_msg = f"❌ Error processing request: {str(e)}"
            logger.error(f"Error in graph stream: {e}", exc_info=True)
            await cl.Message(content=error_msg, author="System").send()
            return

        # Process all chunks
        for chunk in stream_chunks:
            # Yield control to event loop periodically
            await asyncio.sleep(0)
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

        # Stream final response directly from agent output (skip redundant LLM summarizer call)
        logger.info(f"summarizing {len(agent_responses)} agent responses")
        final_response = (
            "\n\n".join(r for r in agent_responses if r.strip())
            or "I processed your request."
        )

        async with cl.Step(name="Response", type="llm") as response_step:
            response_step.input = message.content
            for token in final_response:
                await msg.stream_token(token)
            response_step.output = final_response

    logger.info(f"Final summarized response: {msg.content}")
    logger.info("-" * 100)
    print(f"Total time to process query: {time.time() - debug_start_time}")
    await msg.update()
