# bandu
Bandu: AI Agents based on ROS2

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![CI Status](https://github.com/sachinkum0009/bandu/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/sachinkum0009/bandu/actions)

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
# create a virtual env and install deps
pip install -r requirements.txt

# start app
chainlit run app.py --host 0.0.0.0
```


## UI

![App UI](docs/media/app_ui.png)

## Image Credits

Images used in this repository are downloaded from [Flaticon](https://www.flaticon.com/).
