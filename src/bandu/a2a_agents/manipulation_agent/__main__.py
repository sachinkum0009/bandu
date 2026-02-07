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

"""Manipulation Agent Server - Entry point to start the manipulation agent."""

import logging

import uvicorn

from a2a.server.apps import A2AStarletteApplication
from a2a.server.request_handlers import DefaultRequestHandler
from a2a.server.tasks import InMemoryTaskStore

from .agent_executor import ManipulatorAgentExecutor

from bandu.a2a_agents.skills import Skill, get_skill
from bandu.a2a_agents.cards import get_agent_card, Card

import rclpy
from rai.communication.ros2 import ROS2Connector


# Configure logging to show INFO level messages
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)

logger = logging.getLogger(__name__)


def main():
    ## Initialize ROS2
    rclpy.init()
    connector = None
    try:
        connector = ROS2Connector(executor_type="single_threaded")
        node = connector.node
        node.declare_parameter("conversion_ratio", 1.0)

        # Define the agent skill
        skill_1 = get_skill(Skill.MOVE_TO_POINT)
        skill_2 = get_skill(Skill.MOVE_OBJECT_FROM_TO)
        skill_3 = get_skill(Skill.GET_OBJECT_POSITIONS)
        skill_4 = get_skill(Skill.RESET_ARM)

        # Define the agent card
        agent_card = get_agent_card(
            Card.MANIPULATION_AGENT, [skill_1, skill_2, skill_3, skill_4]
        )

        # Create request handler with the agent executor
        request_handler = DefaultRequestHandler(
            agent_executor=ManipulatorAgentExecutor(connector),
            task_store=InMemoryTaskStore(),
        )

        # Create and run the server
        server = A2AStarletteApplication(
            agent_card=agent_card,
            http_handler=request_handler,
        )

        logger.info("Starting Manipulation Agent on port 9003...")
        uvicorn.run(server.build(), host="0.0.0.0", port=9003)

    except Exception as e:
        logger.error(f"Error starting Manipulation Agent: {e}")
    except KeyboardInterrupt:
        logger.info("Shutting down Manipulation Agent...")
    finally:
        if connector:
            connector.shutdown()
        rclpy.shutdown()

        logger.info("Manipulator Agent shutdown complete.")


if __name__ == "__main__":
    main()
