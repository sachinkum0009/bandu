# bandu
Bandu: AI Agents based on ROS2

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://github.com/sachinkum0009/bandu/blob/main/LICENSE)
[![Docs Status](https://github.com/sachinkum0009/bandu/actions/workflows/docs.yml/badge.svg?branch=main)](https://github.com/sachinkum0009/bandu/actions)
![CodeRabbit Pull Request Reviews](https://img.shields.io/coderabbit/prs/github/sachinkum0009/bandu?utm_source=oss&utm_medium=github&utm_campaign=sachinkum0009%2Fbandu&labelColor=171717&color=FF570A&link=https%3A%2F%2Fcoderabbit.ai&label=CodeRabbit+Reviews)

## Overview

It is a multi agent based application where each agent has different capabilities and tools to interact with the robot using ROS2 communication. The agents can be used to perform various tasks such as driving the robot, getting the temperature of the robot, telling jokes, etc.

![Architecture](docs/media/hierachical_agents.png)

## Features
- Multi-agent system with different capabilities
- Agents can interact with the robot using ROS2 communication
- Agents can use tools to perform various tasks
- Web-based UI to interact with the agents
- Built using Chainlit for easy deployment and interaction
- LLM model used: GPT-4-Turbo (can be changed in the code to use other models)

## Hierarchical Agents

![Hierarchical Agents](docs/media/hierarchical_graph.png)

## Installation steps

```bash
# use uv to install deps
uv sync

# source ros
source /opt/ros/jazzy/setup.bash # change jazzy with your ros distro

# start app
uv run chainlit run scripts/app.py --host 0.0.0.0
```

### Commands to start Manipulation and Navigation Agents

```bash
# source ros
source /opt/ros/jazzy/setup.bash # change jazzy with your ros distro
# start manipulation
uv run start_manipulation_agent

# start navigation
uv run start_navigation_agent

# manual test for a2a agents
uv run start_manipulation_agent
```

## UI

![App UI](docs/media/app_ui.png)

## Image Credits

Images used in this repository are downloaded from [Flaticon](https://www.flaticon.com/).
