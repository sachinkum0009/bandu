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


from __future__ import annotations

from a2a.client import A2ACardResolver, A2AClient
from a2a.types import MessageSendParams, SendMessageRequest
from a2a.types import Message, TextPart, Part, Role
import httpx
from langchain_core.tools import tool
import logging
from rai import get_llm_model
from rai.agents.langchain.core import create_react_runnable
from rai.communication.ros2 import ROS2Connector
from rai_whoami.models import EmbodimentInfo
from uuid import uuid4

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# AGENT URLs
NAVIGATION_AGENT_URL = "http://asus-pc.netbird.cloud:9002"
MANIPULATION_AGENT_URL = "http://asus-pc.netbird.cloud:9003"


@tool
async def query_navigation_agent(query: str) -> str:
    """
    Query the navigation agent with a natural language command and receive a response.

    :param query: Description
    :type query: str
    :return: Description
    :rtype: str
    """
    return await _query_navigation_agent(NAVIGATION_AGENT_URL, query)


@tool
async def query_manipulation_agent(query: str) -> str:
    """
    Query the manipulation agent with a natural language command and receive a response.

    :param query: Description
    :type query: str
    :return: Description
    :rtype: str
    """
    return await _query_navigation_agent(MANIPULATION_AGENT_URL, query)


async def _query_navigation_agent(agent_url: str, query: str) -> str:
    async with httpx.AsyncClient() as client:
        # Fetch agent card
        resolver = A2ACardResolver(client, agent_url)
        try:
            agent_card = await resolver.get_agent_card()
            logger.info(f"Connected to agent at{agent_url}")
        except Exception as e:
            logger.error(f"Failed to fetch agent card from {agent_url}: {e}")
            return f"Error: Could not connect to agent at {agent_url}"

        # Initialize client
        a2a_client = A2AClient(client, agent_card)

        # Create message
        text_part = TextPart(text=query)
        message = Message(
            role=Role.user,
            parts=[Part(text_part)],
            message_id=uuid4().hex,
        )

        request = SendMessageRequest(
            id=str(uuid4()), params=MessageSendParams(message=message)
        )

        # Send request
        try:
            response = await a2a_client.send_message(request)
            logger.info(f"Received response from agent at {agent_url}")
            result = response.model_dump()
            logger.info(f"Response content: {result}")
            # return response.message.parts[0].text if response.message.parts else "No response"
        except Exception as e:
            logger.error(f"Failed to send message to agent at {agent_url}: {e}")
            return f"Error: Failed to send message to agent at {agent_url}"
        return "No response received from agent"  # TODO: Fix it later to return actual response content


async def create_agent(connector: ROS2Connector):
    llm = get_llm_model(model_type="complex_model", streaming=True)

    # Define tools for the navigation agent
    tools = [query_manipulation_agent, query_navigation_agent]

    embodiment_info = EmbodimentInfo.from_file("embodiments/supervisor_embodiment.json")

    agent = create_react_runnable(
        llm=llm,
        tools=tools,  # type: ignore
        system_prompt=embodiment_info.to_langchain(),
    )

    return agent
