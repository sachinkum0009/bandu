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
            message="Can you tell me what the robot is currently seeing?",
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
    msg = cl.Message(content="", author="Agent")
    full_content = ""
    history.append(message.content)
    for chunk in agent.stream({"messages": history}):
        if "thinker" not in chunk:
            continue
        await msg.stream_token(chunk["thinker"]["messages"][-1].content)
        full_content += chunk["thinker"]["messages"][-1].content
    await msg.update()
