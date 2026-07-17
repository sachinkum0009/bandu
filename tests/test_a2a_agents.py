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

import asyncio
import rclpy
from dotenv import load_dotenv
from bandu.agents.supervisor_agent import create_agent
from rai.communication.ros2 import ROS2Connector

load_dotenv()


async def main():
    rclpy.init()
    connector = ROS2Connector(executor_type="single_threaded")

    # Create the supervisor agent
    supervisor_agent = create_agent(connector)

    # Invoke the agent with a query
    query = "Grab object using robot arm to position x: 1.0, y: 2.0, z: 0.5"
    response = await supervisor_agent.ainvoke({"messages": [query]})

    print(response)

    print("task finished")

    rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
