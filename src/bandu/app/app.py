"""
App
"""

from langchain_core.callbacks import BaseCallbackHandler

from typing import Any


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
