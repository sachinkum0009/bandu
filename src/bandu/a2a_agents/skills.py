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

from a2a.types import AgentSkill
from enum import Enum


class Skill(Enum):
    GET_WEATHER = 1
    NAVIGATE_TO_POSE = 2
    GET_NAVIGATE_TO_POSE_FEEDBACK = 3
    GET_NAVIGATE_TO_POSE_RESULT = 4
    CANCEL_NAVIGATE_TO_POSE = 5
    GET_OCCUPANCY_GRID = 6
    MOVE_TO_POINT = 7
    MOVE_OBJECT_FROM_TO = 8
    GET_OBJECT_POSITIONS = 9
    RESET_ARM = 10


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

        case Skill.MOVE_TO_POINT:
            agent_skill = _create_skill(
                "move_to_point",
                "Move to Point",
                "Guide the robot's end effector to a specific point within the manipulator's operational space. "
                "This tool ensures precise movement to the desired location. "
                "While it confirms successful positioning, please note that it doesn't provide feedback on the "
                "success of grabbing or releasing objects. Use additional sensors or tools for that information.",
                ["move", "end effector", "manipulator", "point"],
                [
                    "Move the arm to this point",
                    "Position the end effector at x, y, z",
                    "Guide the robot to the target point",
                ],
            )
        case Skill.MOVE_OBJECT_FROM_TO:
            agent_skill = _create_skill(
                "move_object_from_to",
                "Move Object From To",
                "Move an object from one point to another. "
                "The tool will grab the object from the first point and then release it at the second point. "
                "The tool will not confirm the success of grabbing or releasing objects. Use additional sensors (e.g. camera) or tools for that information.",
                ["move object", "pick and place", "transfer", "manipulator"],
                [
                    "Move the object from point A to point B",
                    "Pick up the object and place it here",
                    "Transfer the object to this location",
                ],
            )

        case Skill.GET_OBJECT_POSITIONS:
            agent_skill = _create_skill(
                "get_object_positions",
                "Get Object Positions",
                "Retrieve the positions of all objects of a specified type in the target frame. "
                "This tool provides accurate positional data but does not distinguish between different colors of the same object type. "
                "While position detection is reliable, please note that object classification may occasionally be inaccurate.",
                [
                    "object positions",
                    "detect objects",
                    "locate objects",
                    "object detection",
                ],
                [
                    "Get the positions of all cubes",
                    "Where are the objects located?",
                    "Find all objects of type X",
                ],
            )

        case Skill.RESET_ARM:
            agent_skill = _create_skill(
                "reset_arm",
                "Reset Arm",
                "Reset the arm to the initial position. Use when the arm is stuck or when arm obstructs the objects.",
                ["reset", "arm", "initial position", "stuck"],
                [
                    "Reset the arm",
                    "Move the arm to initial position",
                    "The arm is stuck, reset it",
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
