"""
App to integrate ROS2 with AI Agents using RAI

Author: Sachin Kumar
Date: 2025-08-12
"""

import chainlit as cl
from rai import get_llm_model, get_tracing_callbacks
from agents.basic_agent import create_agent
import rclpy
from rai.communication.ros2 import (
    ROS2Connector,
    ROS2HRIConnector,
    ROS2HRIMessage,
    ROS2Message,
)

from agents import AgentType, make_team, create_agent_node, State
from agents.response import ChunkData

from rai.agents.langchain.core.megamind import (
    MegamindState,
    create_megamind,
    Executor,
    get_initial_megamind_state,
)
from langchain_core.tools import BaseTool

from agents.tools import GetRobotTemperatureTool, TellMeAJokeTool, GetROS2ImageTool
from rai.tools.ros2 import ROS2Toolkit
from typing import List

history = []

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

builder = make_team([basic, navigator, manipulator, perception])

graph = builder.compile()

graph.get_graph().draw_mermaid_png(output_file_path="team_graph43.png")

# tools: List[BaseTool] = [
#     GetRobotTemperatureTool(connector=connector),
#     TellMeAJokeTool(connector=connector),
# ]
# llm = get_llm_model(model_type="simple_model", streaming=True)
# basic_executor = Executor(
#     name="basic_executor",
#     llm=llm,
#     tools=tools,
#     system_prompt="You are a helpful assistant that has tools like get_robot_temperature to get the current temperature of the robot.",
# )
# navigation_executor = Executor(
#     name="navigation_executor",
#     llm=llm,
#     tools=[
#         *ROS2Toolkit(connector=connector).get_tools(),
#     ],
#     system_prompt="You are a helpful ros2 assistant that has the capability to call tools to list ros2 topics and service which can be used to list the ros2 topics and also publish them.",
# )
# manipulator_executor = Executor(
#     name="manipulator_executor",
#     llm=llm,
#     tools=tools,
#     system_prompt="You are a helpful assistant that can perform manipulation tasks.",
# )

# llm = get_llm_model(model_type="complex_model", streaming=True)
# graph = create_megamind(
#     megamind_llm=llm,
#     megamind_system_prompt=" You are Megamind, a master coordinator of multiple specialized agents. Your job is to delegate tasks to the appropriate agents based on their expertise and capabilities. You must analyze the user's requests, determine which agent is best suited to handle each part of the request, and coordinate the execution of these tasks to provide a comprehensive response back to the user. Always consider the strengths and limitations of each agent when making your decisions. ",
#     executors=[basic_executor, navigation_executor, manipulator_executor],
#     task_planning_prompt="Write a detailed plan to complete the task",
# )
# graph.get_graph().draw_mermaid_png(output_file_path="megamind_graph2.png")

## summarizer
summarizer_llm = get_llm_model(model_type="simple_model", streaming=True)


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
        )
    ]

@cl.on_app_startup
async def on_app_startup():
    print("Chainlit Starting Up")

@cl.on_chat_start
async def on_chat_start():
    print("Chainlit Started")

@cl.step(type="tool")
async def thinking():
    await cl.sleep(1)
    return "tool result"

@cl.step(type="llm")
async def supervisor():
    await cl.sleep(1)
    return "supervisor result"

@cl.step(type="llm")
async def basic_agent():
    await cl.sleep(1)
    return "basic agent result"

@cl.step(type="llm")
async def manipulator_agent():
    await cl.sleep(1)
    return "manipulator agent result"

@cl.step(type="llm")
async def navigator_agent():
    await cl.sleep(1)
    return "navigator agent result"

@cl.on_message
async def on_message(message: cl.Message):
    print(f"Received message: {message.content}")
    tool_res = await supervisor()
    image = cl.Image(name="robot", path="/media/asus/backup/zzzzz/ros2/rtw_workspaces/rolling_generic_ws/src/bandu/agents/nodes/apple-table.jpg")
    msg = cl.Message(content="", author="Agent")
    await msg.update()
    full_content = ""
    history.append(message.content)
    tool_name = None
    initial_message = get_initial_megamind_state(
        task=message.content
    )
    chunks = []
    agent_responses: List[str] = []
    for chunk in graph.stream({"messages": [message.content]}, config={"callbacks": get_tracing_callbacks()}):
        print(f"Chunk: {chunk}")
        # if "supervisor" in chunk:
        #     continue

        # chunk_data = ChunkData.from_dict(chunk)
        # last_ai_message = chunk_data.get_last_ai_message()
        
        # if last_ai_message:
        #     print(f"last ai message: {last_ai_message}")
        #     # Stream the AI message content to the user
        #     await msg.stream_token(last_ai_message)
        #     full_content += last_ai_message

        # print("-----")
        chunks.append(chunk)
        agent_response = next(iter(chunk.items()))[1]  # .get("messages", [""])
        print(f"Agent Response: {agent_response}")
        
        # Call the appropriate agent function based on 'next' value
        if "next" in agent_response:
            next_agent = agent_response["next"]
            if next_agent == "basic":
                await basic_agent()
            elif next_agent == "navigator":
                await navigator_agent()
            elif next_agent == "manipulator":
                await manipulator_agent()
        
        # Extract content from the agent response
        if "messages" in agent_response and len(agent_response["messages"]) > 0:
            message_content = agent_response["messages"][-1].content
            print(f"Message Content: {message_content}")
            agent_responses.append(message_content)

        
        # # Print tool name and ToolMessage content if tools are used
        # if "tools" in chunk and "messages" in chunk["tools"]:
        #     for m in chunk["tools"]["messages"]:
        #         # If message has 'tool_calls' (AIMessage)
        #         if hasattr(m, 'tool_calls'):
        #             for tool_call in m.tool_calls:
        #                 print(tool_call.get('name', None))
        #         # If message has 'name' (ToolMessage)
        #         elif hasattr(m, 'name'):
        #             tool_name = getattr(m, 'name', None)
        #             content = getattr(m, 'content', None)
        #             print(tool_name)
        #             # print(content)
        #         # If message is a dict
        #         elif isinstance(m, dict):
        #             if 'tool_calls' in m:
        #                 for tool_call in m['tool_calls']:
        #                     print(tool_call.get('name', None))
        #             elif 'name' in m:
        #                 print(m['name'])
        #                 if 'content' in m:
        #                     print(m['content'])
        #     if tool_name == "get_robot_temperature":
        #         print("Get robot temperature tool is used")
        #     elif tool_name == "get_ros2_image":
        #         print("get image tool is used")
        #         # decode content to base64 image and save as tmp image
        #         import base64, tempfile
        #         if content: # type: ignore
        #             img_bytes = base64.b64decode(content)
        #             with tempfile.NamedTemporaryFile(delete=False, suffix=".jpg") as tmp_img:
        #                 tmp_img.write(img_bytes)
        #                 tmp_img_path = tmp_img.name
        #             print(f"Image saved to {tmp_img_path}")
        #             img = cl.Image(name="robot", path=tmp_img_path) # type: ignore
        #             msg = cl.Message(content="", author="Agent", elements=[img])
        #             await msg.stream_token("")
        #             # return
        #     else:
        #         print(f"tool name is {tool_name}")
        # Continue processing other chunks as needed
        # if "thinker" not in chunk:
        #     continue
    # print("out of for loop")
    # response = chunks[-2]
    # print(response)
    # key, value = next(iter(response.items()))
    # final_response = value["messages"][-1].content
    # print(f"Final Response from chunks: {final_response}")

    print(f"summarizing {len(agent_responses)} agent responses")
    prompt = "please read the following responses and provide a brief response to the user. "
    summary_prompt = prompt + message.content + " ".join(agent_responses)
    
    # Stream the summarized response
    async for chunk in summarizer_llm.astream(summary_prompt):
        if hasattr(chunk, 'content') and chunk.content:
            content = chunk.content if isinstance(chunk.content, str) else str(chunk.content)
            await msg.stream_token(content)
    
    print(f"Final summarized response: {msg.content}")
    print("-"*100)
    await msg.update()
