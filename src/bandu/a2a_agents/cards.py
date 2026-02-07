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

from a2a.types import AgentCapabilities, AgentCard, AgentSkill
from enum import Enum


class Card(Enum):
    WEATHER_AGENT = 0
    NAVIGATION_AGENT = 1
    MANIPULATION_AGENT = 2
    PERCEPTION_AGENT = 3


def get_agent_card(card: Card, skills: list[AgentSkill]) -> AgentCard:
    match card:
        case Card.WEATHER_AGENT:
            agent_card = _create_agent_card(
                name="Weather Agent",
                description="An agent that provides simulated weather information for cities around the world",
                url="http://localhost:9001/",
                skills=skills,
                version="1.0.0",
                default_input_modes=["text"],
                default_output_modes=["text"],
                supports_authenticated_extended_card=False,
            )
        case Card.NAVIGATION_AGENT:
            agent_card = _create_agent_card(
                name="Navigation Agent",
                description="An agent that can navigate the robot to different target locations",
                url="http://localhost:9002",
                skills=skills,
                version="1.0.0",
                default_input_modes=["text"],
                default_output_modes=["text"],
                supports_authenticated_extended_card=False,
            )
        case Card.MANIPULATION_AGENT:
            agent_card = _create_agent_card(
                name="Manipulation Agent",
                description="An agent that can control the robot arm to perform manipulation tasks",
                url="http://localhost:9003",
                skills=skills,
                version="1.0.0",
                default_input_modes=["text"],
                default_output_modes=["text"],
                supports_authenticated_extended_card=False,
            )
        case Card.PERCEPTION_AGENT:
            agent_card = _create_agent_card(
                name="Perception Agent",
                description="An agent that can perceive the environment and provide information about objects and surroundings",
                url="http://localhost:9004",
                skills=skills,
                version="1.0.0",
                default_input_modes=["text"],
                default_output_modes=["text"],
                supports_authenticated_extended_card=False,
            )

    return agent_card


def _create_agent_card(
    name: str,
    description: str,
    url: str,
    version: str,
    skills: list[AgentSkill],
    default_input_modes: list[str],
    default_output_modes: list[str],
    streaming: bool = True,
    supports_authenticated_extended_card: bool = False,
) -> AgentCard:
    if default_input_modes is None:
        default_input_modes = ["text"]
    if default_output_modes is None:
        default_output_modes = ["text"]

    agent_card = AgentCard(
        name=name,
        description=description,
        url=url,
        version=version,
        default_input_modes=default_input_modes,
        default_output_modes=default_output_modes,
        capabilities=AgentCapabilities(streaming=streaming),
        skills=skills,
        supports_authenticated_extended_card=supports_authenticated_extended_card,
    )
    return agent_card
