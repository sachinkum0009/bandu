from a2a.types import AgentSkill
from enum import Enum


class Skill(Enum):
    GET_WEATHER = 1
    NAVIGATE_TO_POSE = 2
    GET_NAVIGATE_TO_POSE_FEEDBACK = 3
    GET_NAVIGATE_TO_POSE_RESULT = 4
    CANCEL_NAVIGATE_TO_POSE = 5
    GET_OCCUPANCY_GRID = 6


def get_skill(skill: Skill) -> AgentSkill:
    match skill:
        case Skill.GET_WEATHER:
            agent_skill = _create_skill(
                "get_weather",
                "Get Weather Information",
                "Returns simulaed wather data for any city",
                ["weather", "forecast", "temperature"],
                [
                    "What's the weather in Paris?",
                    "Weather forecast for London",
                    "Is it sunny in Tokyo?",
                ],
            )

        case Skill.NAVIGATE_TO_POSE:
            agent_skill = _create_skill(
                "navigate_to_pose",
                "Navigate to a specific pose",
                "Navigates the robot to a specific 2D target location",
                ["navigate", "go to", "location"],
                [
                    "Navigate to particlar location",
                    "Navigate to x, y position",
                    "Navigate to this target goal",
                ],
            )

        case Skill.GET_NAVIGATE_TO_POSE_FEEDBACK:
            agent_skill = _create_skill(
                "get_navigate_to_pose_feedback",
                "Get Navigate to Pose Feedback",
                "Get the feedback of the navigate to pose action",
                ["feedback", "navigation status", "progress"],
                [
                    "What's the navigation feedback?",
                    "Get navigation progress",
                    "Check navigation status",
                ],
            )

        case Skill.GET_NAVIGATE_TO_POSE_RESULT:
            agent_skill = _create_skill(
                "get_navigate_to_pose_result",
                "Get Navigate to Pose Result",
                "Get the result of the navigate to pose action",
                ["result", "navigation result", "completion"],
                [
                    "What's the navigation result?",
                    "Did the navigation complete?",
                    "Get navigation outcome",
                ],
            )

        case Skill.CANCEL_NAVIGATE_TO_POSE:
            agent_skill = _create_skill(
                "cancel_navigate_to_pose",
                "Cancel Navigate to Pose",
                "Cancel the navigate to pose action",
                ["cancel", "stop navigation", "abort"],
                [
                    "Cancel the navigation",
                    "Stop navigating",
                    "Abort the current navigation",
                ],
            )

        case Skill.GET_OCCUPANCY_GRID:
            agent_skill = _create_skill(
                "get_occupancy_grid",
                "Get Occupancy Grid Map",
                "Get the current map as an image with the robot's position marked on it",
                ["map", "occupancy grid", "robot position"],
                [
                    "Show me the map",
                    "Get the current map",
                    "Where is the robot on the map?",
                ],
            )

        case _:
            agent_skill = _get_dummy_skill()

    return agent_skill


def _create_skill(
    id: str, name: str, description: str, tags: list[str], examples: list[str]
) -> AgentSkill:
    return AgentSkill(
        id=id,
        name=name,
        description=description,
        tags=tags,
        examples=examples,
    )


def _get_dummy_skill() -> AgentSkill:
    return _create_skill(
        "get_weather",
        "Get Weather Information",
        "Returns simulaed wather data for any city",
        ["weather", "forecast", "temperature"],
        [
            "What's the weather in Paris?",
            "Weather forecast for London",
            "Is it sunny in Tokyo?",
        ],
    )
