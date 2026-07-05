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


from typing import Any

from langchain_core.callbacks import BaseCallbackHandler


# Custom callback to track tool usage
class ToolTrackingCallback(BaseCallbackHandler):
    def __init__(self):
        self.tool_calls = []
        self.tool_results = []

    def on_tool_start(
        self, serialized: dict[str, Any], input_str: str, **kwargs
    ) -> None:
        tool_name = serialized.get("name", "Unknown Tool")
        self.tool_calls.append({"name": tool_name, "input": input_str})
        print(f"[ToolTracker] Tool started: {tool_name}")

    def on_tool_end(self, output: str, **kwargs) -> None:
        self.tool_results.append({"output": output})
        print(f"[ToolTracker] Tool ended with output: {output}")
