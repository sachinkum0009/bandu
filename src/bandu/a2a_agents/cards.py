from a2a.types import AgentCapabilities, AgentCard, AgentSkill
from enum import Enum


class Card(Enum):
    WEATHER_AGENT = 0
    NAVIGATION_AGENT = 1


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
