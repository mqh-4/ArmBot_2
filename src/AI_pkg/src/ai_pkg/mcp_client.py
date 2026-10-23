# MCP Client 示例：连接 mcp_server.py 并调用 add 工具
# 依赖安装：pip3 install --user fastmcp
import asyncio
import os

from fastmcp import Client
from fastmcp.client.transports import StdioTransport

SERVER_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mcp_server.py")


async def main():
    # 1. 创建 stdio 传输：客户端会把 mcp_server.py 作为子进程拉起，
    #    通过标准输入输出与它交换 MCP 协议消息
    transport = StdioTransport("python3", [SERVER_PATH])

    # 2. 创建客户端并连接
    async with Client(transport) as client:
        # 3. 发现服务器提供的工具
        tools = await client.list_tools()
        print("可用工具：", [t.name for t in tools])

        # 4. 调用 add 工具
        result = await client.call_tool("add", {"a": 100, "b": 50})
        print(f"add(100, 50) = {result.content[0].text}")


if __name__ == "__main__":
    asyncio.run(main())
