"""
App to integrate ROS2 with AI Agents using RAI

Author: Sachin Kumar
Date: 2025-08-12
"""

import chainlit as cl
from agents.basic_agent import create_agent

history = []

agent = create_agent()

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

@cl.on_message
async def on_message(message: cl.Message):
    image = cl.Image(name="robot", path="/media/asus/backup/zzzzz/ros2/rtw_workspaces/rolling_ws/src/bandu/agents/nodes/apple-table.jpg")
    msg = cl.Message(content="", author="Agent")
    full_content = ""
    history.append(message.content)
    tool_name = None
    for chunk in agent.stream({"messages": history}):
        # print(f"chunk data {chunk}")
        # Print tool name and ToolMessage content if tools are used
        if "tools" in chunk and "messages" in chunk["tools"]:
            for m in chunk["tools"]["messages"]:
                # If message has 'tool_calls' (AIMessage)
                if hasattr(m, 'tool_calls'):
                    for tool_call in m.tool_calls:
                        print(tool_call.get('name', None))
                # If message has 'name' (ToolMessage)
                elif hasattr(m, 'name'):
                    tool_name = getattr(m, 'name', None)
                    content = getattr(m, 'content', None)
                    print(tool_name)
                    # print(content)
                # If message is a dict
                elif isinstance(m, dict):
                    if 'tool_calls' in m:
                        for tool_call in m['tool_calls']:
                            print(tool_call.get('name', None))
                    elif 'name' in m:
                        print(m['name'])
                        if 'content' in m:
                            print(m['content'])
            if tool_name == "get_robot_temperature":
                print("Get robot temperature tool is used")
            elif tool_name == "get_ros2_image":
                print("get image tool is used")
                # decode content to base64 image and save as tmp image
                import base64, tempfile
                if content:
                    img_bytes = base64.b64decode(content)
                    with tempfile.NamedTemporaryFile(delete=False, suffix=".jpg") as tmp_img:
                        tmp_img.write(img_bytes)
                        tmp_img_path = tmp_img.name
                    print(f"Image saved to {tmp_img_path}")
                    img = cl.Image(name="robot", path=tmp_img_path) # type: ignore
                    msg = cl.Message(content="", author="Agent", elements=[img])
                    await msg.stream_token("")
                    # return
            else:
                print(f"tool name is {tool_name}")
        if "thinker" not in chunk:
            continue
        # if tool_name == "get_ros2_image":
        #     await msg.stream_token("Pleaase check image")
        # else:
        #     await msg.stream_token(chunk["thinker"]["messages"][-1].content)
        await msg.stream_token(chunk["thinker"]["messages"][-1].content)
        full_content += chunk["thinker"]["messages"][-1].content
    await msg.update()
