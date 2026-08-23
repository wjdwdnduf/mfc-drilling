# MCP function library (`kuka.py`)

`kuka.py` implements the modular function call (MFC) layer described in the paper. It is an
extension module for [robotmcp/ros-mcp-server](https://github.com/robotmcp/ros-mcp-server)
(Apache-2.0), not a standalone MCP server.

Every tool in this file is intentionally thin: it publishes one string on `/kuka/command`
and returns immediately. All robot-side logic lives in `kuka_task_client/kuka_topic_mover.py`.
This separation is the point — the LLM chooses *which* verified routine runs, never *how* it runs.

## Install

```bash
git clone https://github.com/robotmcp/ros-mcp-server.git
cd ros-mcp-server
cp /path/to/this/repo/mcp/kuka.py ros_mcp/tools/kuka.py
```

Register it in `ros_mcp/tools/__init__.py`:

```python
from ros_mcp.tools.kuka import register_kuka_tools   # add this import

def register_all_tools(mcp, ws_manager, rosbridge_ip="127.0.0.1", rosbridge_port=9090):
    ...
    register_kuka_tools(mcp, ws_manager)             # add this call
```

Then install and launch:

```bash
uv sync                       # or: pip install -e .
bash launch/launch_mcp_server.sh
```

## Connect an MCP client

Claude Desktop — `claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "uv",
      "args": ["--directory", "/absolute/path/to/ros-mcp-server", "run", "server.py"]
    }
  }
}
```

The ROS side must be reachable through rosbridge before the tools will do anything:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## Tool reference

| Tool | Command published on `/kuka/command` |
| --- | --- |
| `check_robot_status()` | — (reads `/kuka/status`) |
| `send_robot_command(command)` | `scan` \| `global_workbenchN` \| `local_workbenchN` |
| `save_camera_image(camera)` | `save_image:camera_1` \| `save_image:camera_2` |
| `detect_rectangles()` | `detect` |
| `center_line()` | `center_line` |
| `mark_point(x, y)` | `mark_point:x:y` |
| `adjust_mark(direction, pixels, mark_index)` | `adjust_point:dir:px[:idx]` |
| `insert_mark(a, b)` | `insert_mark:a:b` |
| `delete_mark(idx)` | `delete_mark:idx` |
| `save_points()` | `save_point` |
| `list_points()` | `list_points` |
| `clear_points()` | `clear_points` |
| `go_to_point(idx)` | `go_to_point:idx` |
| `go_to_all()` | `go_to_all` |
| `go_to_sequence(seq)` | `go_to_seq:i:j:k` |
| `confirm_and_move()` | `confirm_and_move` |
| `drill_point(idx, depth_mm)` | `drill_point:idx:depth` |
| `drill_sequence(seq, depth_mm)` | `drill_seq:depth:i:j:k` |

## Safety notes

- `send_robot_command` and the drilling tools are annotated `destructiveHint=True`; keep
  human confirmation enabled in the MCP client for these.
- Workbench approach is a **mandatory two-step sequence** (`global_workbenchN` then
  `local_workbenchN`), with a status check in between. Skipping the second step leaves the
  robot coarsely positioned.
- The spindle is driven by `drill_control_server` in `kuka_eki`, which is a separate ROS 2
  service and is not directly exposed as an MCP tool.
