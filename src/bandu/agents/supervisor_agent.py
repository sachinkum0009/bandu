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
import asyncio
import httpx
from langchain_core.tools import tool
import logging
from importlib import resources
import os
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
NAVIGATION_AGENT_URL = os.getenv("NAVIGATION_AGENT_URL", "")
MANIPULATION_AGENT_URL = os.getenv("MANIPULATION_AGENT_URL", "http://localhost:9003")


@tool
def query_navigation_agent(query: str) -> str:
    """
    Send a natural-language command to the navigation agent and return its textual response.

    Parameters:
        query (str): Natural-language command or question to send to the navigation agent.

    Returns:
        str: The navigation agent's response text, or an error message describing a failure to contact or communicate with the agent.
    """
    return _query_agent(NAVIGATION_AGENT_URL, query)


@tool
def query_manipulation_agent(query: str) -> str:
    """
    Send a natural-language command to the manipulation agent and return its textual response.

    Parameters:
        query (str): Natural language command to send to the manipulation agent.

    Returns:
        str: The manipulation agent's response text, or an error message if the request failed.
    """
    logger.info("query manipulation agent tool is invoked")
    logger.info(f"manipulation agent url: {MANIPULATION_AGENT_URL}")
    return _query_agent(MANIPULATION_AGENT_URL, query)


async def _query_agent_async(agent_url: str, query: str) -> str:
    """
    Async implementation to send a natural-language query to a remote A2A agent.

    Parameters:
        agent_url (str): The base URL of the target A2A agent.
        query (str): The natural-language query to send to the agent.

    Returns:
        str: The agent's response text if available, or an error message describing the failure.
    """
    # Set longer timeout for agent-to-agent communication (agents can take time to process)
    timeout = httpx.Timeout(120.0, connect=10.0)
    async with httpx.AsyncClient(timeout=timeout) as client:
        # Fetch agent card
        resolver = A2ACardResolver(client, agent_url)
        try:
            agent_card = await resolver.get_agent_card()
            logger.info(f"Connected to agent at {agent_url}")
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

            # Extract text from response - handle both dict and object formats
            text_parts = []

            # Try to access as object attributes
            if hasattr(response, "result") and response.result:
                message_result = response.result
                if hasattr(message_result, "parts") and message_result.parts:
                    for part in message_result.parts:
                        # Handle both object attributes and dict keys
                        if isinstance(part, dict):
                            if "text" in part and part["text"]:
                                text_parts.append(part["text"])
                        elif hasattr(part, "text") and part.text:
                            text_parts.append(part.text)

            # Fallback: try accessing from the dumped dict
            if not text_parts and "result" in result:
                result_data = result["result"]
                if "parts" in result_data and result_data["parts"]:
                    for part in result_data["parts"]:
                        if isinstance(part, dict) and "text" in part and part["text"]:
                            text_parts.append(part["text"])

            if text_parts:
                return " ".join(text_parts)
            return "No response text in agent reply"
        except Exception as e:
            logger.error(f"Failed to send message to agent at {agent_url}: {e}")
            return f"Error: Failed to send message to agent at {agent_url}"


def _query_agent(agent_url: str, query: str) -> str:
    """
    Send a natural-language query to a remote A2A agent and return the agent's response or an error message.

    This function connects to the agent at agent_url, constructs and sends a user message containing query, and returns the agent's textual response. On failure to retrieve the agent card or to send the message, it returns an error string describing the failure. Note: when a request succeeds but no agent text is extracted yet, the function currently returns a placeholder string.

    Parameters:
        agent_url (str): The base URL of the target A2A agent.
        query (str): The natural-language query to send to the agent.

    Returns:
        str: The agent's response text if available, or an error message describing the failure (currently may return a placeholder string when no response content is extracted).
    """
    # Run the async function in a new event loop
    return asyncio.run(_query_agent_async(agent_url, query))


def create_agent(connector: ROS2Connector):
    """
    Create and return a React-style runnable agent configured for the supervisor embodiment.

    The agent is initialized with a complex streaming language model, registers the navigation and manipulation query tools, and uses the system prompt derived from the supervisor embodiment JSON.

    Returns:
        agent: A runnable React-style agent configured with the LLM, the registered tools, and the embodiment-derived system prompt.

    """

    llm = get_llm_model(model_type="complex_model", streaming=True)

    # Define tools for the navigation agent
    tools = [query_manipulation_agent]

    embodiment_path = resources.files("bandu").joinpath(
        "../../embodiments/supervisor_embodiment.json"
    )
    embodiment_info = EmbodimentInfo.from_file(str(embodiment_path))

    agent = create_react_runnable(
        llm=llm,
        tools=tools,  # type: ignore
        system_prompt=embodiment_info.to_langchain(),
    )

    return agent
