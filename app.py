"""
App to integrate ROS2 with AI Agents using RAI

Author: Sachin Kumar
Date: 2025-08-12
"""

import chainlit as cl
from agents.basic_agent import create_agent

agent = create_agent()

@cl.on_app_startup
async def on_app_startup():
    print("Chainlit Starting Up")

@cl.on_chat_start
async def on_chat_start():
    print("Chainlit Started")

@cl.on_message
async def on_message(message: cl.Message):
    # Show spinner while agent is thinking
    msg = cl.Message(content="", author="Agent")
    # Stream agent response
    full_content = ""
    for chunk in agent.stream({"messages": [message.content]}):
        await msg.stream_token(chunk["thinker"]["messages"][-1].content)
        full_content += chunk["thinker"]["messages"][-1].content
    await msg.update()
