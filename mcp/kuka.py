"""KUKA Robot Async tools for ROS MCP."""
import json
import time
from fastmcp import FastMCP
from mcp.types import ToolAnnotations
from ros_mcp.utils.websocket import WebSocketManager


def register_kuka_tools(mcp: FastMCP, ws_manager: WebSocketManager) -> None:
    """Register KUKA robot control tools."""

    # =============================================
    # Status Check
    # =============================================
    @mcp.tool(
        description="Check the robot's current task status and result data via /kuka/status. Always verify 'STATUS: SUCCESS' and the accompanying 'DATA' before proceeding to the next command."
    )
    def check_robot_status():
        return True

    # =============================================
    # Scan & Move
    # =============================================
    @mcp.tool(
        description="""Move the robot to a workbench. This is the ONLY tool for workbench navigation.
        When the user says 'move to workbench 1' or 'go to workbench 2', ALWAYS use this tool.

        [Available Commands]
        - 'scan': Scan for nearby workbenches using the global camera.
        - 'global_workbench1' or 'global_workbench2': Coarse approach using global camera tag.
        - 'local_workbench1' or 'local_workbench2': Precise re-alignment using local camera tag.

        [MANDATORY 2-STEP WORKFLOW for workbench navigation]
        You MUST follow this exact sequence:
        1. send_robot_command('global_workbenchN') → check_robot_status (wait for SUCCESS)
        2. send_robot_command('local_workbenchN')  → check_robot_status (wait for SUCCESS)
        Step 1 uses the global camera for coarse positioning.
        Step 2 uses the local camera for precise alignment (the local tag becomes visible ONLY AFTER step 1).
        NEVER skip step 2. NEVER combine both into one call. ALWAYS check status between steps.

        [IMPORTANT]
        - 'workbench' commands go here, NOT to go_to_point.
        - go_to_point is ONLY for saved drill/task points (P1, P2, ...), never for workbenches.
        - Always run 'scan' first to identify visible workbenches before moving.
        - After both steps succeed, you can proceed with detect, center_line, etc.""",
        annotations=ToolAnnotations(
            title="Move to Workbench",
            destructiveHint=True,
        ),
    )
    def send_robot_command(command: str) -> dict:
        ws_manager.send({
            "op": "publish",
            "topic": "/kuka/command",
            "msg": {"data": command}
        })
        return {"result": f"Command '{command}' sent. Use check_robot_status to verify the result."}

    # =============================================
    # Image Save
    # =============================================
    @mcp.tool(
        description="""Save a camera image to disk.
        - camera: 'camera_1' (end effector camera) or 'camera_2' (global camera)
        Images are saved to ~/kuka_images/"""
    )
    def save_camera_image(camera: str) -> dict:
        ws_manager.send({
            "op": "publish",
            "topic": "/kuka/command",
            "msg": {"data": f"save_image:{camera}"}
        })
        return {"result": f"Image save command for {camera} sent. Use check_robot_status to get the file path."}

    # =============================================
    # Rectangle Detection
    # =============================================
    @mcp.tool(
        description="""Automatically detect orange rectangles in the end effector camera image.
        Detected rectangle centers become pending marks (M1, M2, ...).
        The result image with yellow outlines and red center points is displayed automatically.
        After detection, use center_line, adjust_mark, insert_mark, or delete_mark to edit."""
    )
    def detect_rectangles() -> dict:
        ws_manager.send({
            "op": "publish",
            "topic": "/kuka/command",
            "msg": {"data": "detect"}
        })
        return {"result": "Rectangle detection command sent. Use check_robot_status to see results."}

    # =============================================
    # Center Line Generation
    # =============================================
    @mcp.tool(
        description="""Generate center-line points from detected rectangles.
        Splits pending marks into top and bottom rows, matches pairs by closest X-coordinate,
        and computes the midpoint of each pair.
        Existing marks are replaced with the center-line points.
        Must run detect_rectangles first."""
    )
    def center_line() -> dict:
        ws_manager.send({
            "op": "publish",
            "topic": "/kuka/command",
            "msg": {"data": "center_line"}
        })
        return {"result": "Center line generation command sent. Use check_robot_status to see results."}

    # =============================================
    # Mark Editing
    # =============================================
    @mcp.tool(
        description="""Add a pending mark (red dot) at a specific pixel location on the latest camera image.
        Marks accumulate (are not overwritten).
        - x: horizontal pixel coordinate
        - y: vertical pixel coordinate"""
    )
    def mark_point(x: int, y: int) -> dict:
        ws_manager.send({
            "op": "publish",
            "topic": "/kuka/command",
            "msg": {"data": f"mark_point:{x}:{y}"}
        })
        return {"result": f"Mark added at ({x},{y}). Check the displayed image for confirmation."}

    @mcp.tool(
        description="""Adjust the position of a pending mark by a given number of pixels.
        - direction: 'up', 'down', 'left', or 'right'
        - pixels: number of pixels to move
        - mark_index: which mark to adjust (M1=1, M2=2, ...). If omitted, adjusts the last mark."""
    )
    def adjust_mark(direction: str, pixels: int, mark_index: int = None) -> dict:
        if mark_index is not None:
            cmd = f"adjust_point:{direction}:{pixels}:{mark_index}"
        else:
            cmd = f"adjust_point:{direction}:{pixels}"
        ws_manager.send({
            "op": "publish",
            "topic": "/kuka/command",
            "msg": {"data": cmd}
        })
        target = f"M{mark_index}" if mark_index else "the last mark"
        return {"result": f"Moved {target} {direction} by {pixels}px. Check the displayed image."}

    @mcp.tool(
        description="""Insert a midpoint between two existing pending marks.
        - mark_a: first mark index
        - mark_b: second mark index
        Example: insert_mark(2, 7) inserts the midpoint of M2 and M7 as a new mark."""
    )
    def insert_mark(mark_a: int, mark_b: int) -> dict:
        ws_manager.send({
            "op": "publish",
            "topic": "/kuka/command",
            "msg": {"data": f"insert_mark:{mark_a}:{mark_b}"}
        })
        return {"result": f"Midpoint between M{mark_a} and M{mark_b} inserted. Use check_robot_status to confirm."}

    @mcp.tool(
        description="""Delete a specific pending mark.
        - mark_index: mark number to delete (M1=1, M2=2, ...)
        Remaining marks are automatically re-indexed."""
    )
    def delete_mark(mark_index: int) -> dict:
        ws_manager.send({
            "op": "publish",
            "topic": "/kuka/command",
            "msg": {"data": f"delete_mark:{mark_index}"}
        })
        return {"result": f"M{mark_index} deleted. Use check_robot_status to confirm."}

    # =============================================
    # Point Save & Management
    # =============================================
    @mcp.tool(
        description="""Convert all pending marks (M) to saved points (P) in base_link coordinates.
        Uses depth camera and TF to compute 3D positions.
        Once saved, points can be used with go_to_point, drill_point, etc. without requiring tag visibility.
        Must have pending marks from detect/center_line/mark_point before calling."""
    )
    def save_points() -> dict:
        ws_manager.send({
            "op": "publish",
            "topic": "/kuka/command",
            "msg": {"data": "save_point"}
        })
        return {"result": "Save points command sent. Use check_robot_status to see saved point details."}

    @mcp.tool(
        description="List all saved points (P1, P2, ...) with their base_link coordinates."
    )
    def list_points() -> dict:
        ws_manager.send({
            "op": "publish",
            "topic": "/kuka/command",
            "msg": {"data": "list_points"}
        })
        return {"result": "List points command sent. Use check_robot_status to see the list."}

    @mcp.tool(
        description="Delete all saved points."
    )
    def clear_points() -> dict:
        ws_manager.send({
            "op": "publish",
            "topic": "/kuka/command",
            "msg": {"data": "clear_points"}
        })
        return {"result": "All saved points cleared."}

    # =============================================
    # Point Navigation
    # =============================================
    @mcp.tool(
        description="""Move the robot to a saved task point (P1, P2, ...). No tag visibility required.
        These are points created via detect → center_line → save_points workflow.
        NOT for workbench navigation — use send_robot_command('global_workbenchN') for that.
        - point_index: saved point number (P1=1, P2=2, ...)"""
    )
    def go_to_point(point_index: int) -> dict:
        ws_manager.send({
            "op": "publish",
            "topic": "/kuka/command",
            "msg": {"data": f"go_to_point:{point_index}"}
        })
        return {"result": f"Moving to P{point_index}. Use check_robot_status to confirm arrival."}

    @mcp.tool(
        description="Move the robot to all saved points in order (P1 → P2 → P3 → ...)."
    )
    def go_to_all() -> dict:
        ws_manager.send({
            "op": "publish",
            "topic": "/kuka/command",
            "msg": {"data": "go_to_all"}
        })
        return {"result": "Sequential move to all points started. Use check_robot_status to monitor progress."}

    @mcp.tool(
        description="""Move the robot to saved task points in a custom order. Duplicates and repeats are allowed.
        These are points created via detect → center_line → save_points, NOT workbenches.
        - sequence: list of saved point indices. Example: [3, 1, 2] → P3 → P1 → P2"""
    )
    def go_to_sequence(sequence: list[int]) -> dict:
        seq_str = ":".join(str(i) for i in sequence)
        ws_manager.send({
            "op": "publish",
            "topic": "/kuka/command",
            "msg": {"data": f"go_to_seq:{seq_str}"}
        })
        order = " → ".join(f"P{i}" for i in sequence)
        return {"result": f"Moving in order: {order}. Use check_robot_status to monitor progress."}

    # =============================================
    # Drilling
    # =============================================
    @mcp.tool(
        description="""Perform drilling at a saved task point (P1, P2, ...).
        Executes a 3-step process: approach (10cm) → drill down (specified depth) → retract (10cm).
        Points must be created via detect → center_line → save_points before drilling.
        - point_index: saved point number to drill at (P1=1, P2=2, ...)
        - depth_mm: drilling depth in millimeters (default: 5)"""
    )
    def drill_point(point_index: int, depth_mm: int = 5) -> dict:
        ws_manager.send({
            "op": "publish",
            "topic": "/kuka/command",
            "msg": {"data": f"drill_point:{point_index}:{depth_mm}"}
        })
        return {"result": f"Drilling at P{point_index} ({depth_mm}mm) started. Use check_robot_status to monitor progress."}

    @mcp.tool(
        description="""Perform drilling at saved task points in a custom order. Duplicates and repeats are allowed.
        Each point goes through approach → drill (specified depth) → retract before moving to the next.
        Points must be created via detect → center_line → save_points before drilling.
        - sequence: list of saved point indices. Example: [3, 1, 2] → drill P3 → drill P1 → drill P2
        - depth_mm: drilling depth in millimeters (default: 5)"""
    )
    def drill_sequence(sequence: list[int], depth_mm: int = 5) -> dict:
        seq_str = ":".join(str(i) for i in sequence)
        ws_manager.send({
            "op": "publish",
            "topic": "/kuka/command",
            "msg": {"data": f"drill_seq:{depth_mm}:{seq_str}"}
        })
        order = " → ".join(f"P{i}" for i in sequence)
        return {"result": f"Drilling in order ({depth_mm}mm): {order}. Use check_robot_status to monitor progress."}

    # =============================================
    # Immediate Move (without saving)
    # =============================================
    @mcp.tool(
        description="""Move the robot immediately to the last pending mark position without saving it.
        Use after mark_point and adjust_mark when the user confirms the position."""
    )
    def confirm_and_move() -> dict:
        ws_manager.send({
            "op": "publish",
            "topic": "/kuka/command",
            "msg": {"data": "confirm_and_move"}
        })
        return {"result": "Moving to confirmed position. Use check_robot_status to verify arrival."}