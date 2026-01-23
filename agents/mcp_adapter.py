from langchain_core.tools import BaseTool
from langchain_mcp_adapters.client import MultiServerMCPClient
from langchain_mcp_adapters.sessions import Connection, StreamableHttpConnection, StdioConnection

async def get_mcp_adapter_tools() -> list[BaseTool]:
    connections: dict[str, Connection] = {}
    connections["math"] = StdioConnection({
        "transport": "stdio",
        "command": "python",
        "args": ["/path/to/math_server.py"]
    })
    connections["weather"] = StreamableHttpConnection({
        "url": "http://localhost:8000/mcp",
        "transport": "streamable_http"
    })

    client = MultiServerMCPClient(connections)
    tools = await client.get_tools()  
    return tools