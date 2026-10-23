# MCP Server 示例：提供 add 加法工具
# 依赖安装：pip3 install --user fastmcp
from fastmcp import FastMCP

# 1. 创建 MCP 服务器实例
mcp = FastMCP(
    name="math-server",       # 服务器名称，客户端可见
    instructions="一个简单的数学工具服务器，提供加法运算",
)


# 2. 用 @mcp.tool 装饰器注册工具：函数名即工具名，docstring 即工具描述
@mcp.tool
def add(a: int, b: int) -> int:
    """计算两个整数的和"""
    return a + b


# 3. 以 stdio 模式启动服务器（客户端通过标准输入输出与本进程通信）
if __name__ == "__main__":
    mcp.run()  # 默认 transport="stdio"
