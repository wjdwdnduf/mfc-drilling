# Third-party components

This repository contains only code written by the authors. The components below are **not**
vendored here; they are fetched at setup time via `ros2_ws.repos` (`vcs import`) or `pip`,
and remain under their own licenses.

| Component | Upstream | License | Role in this work |
| --- | --- | --- | --- |
| `ros-mcp-server` | https://github.com/robotmcp/ros-mcp-server | Apache-2.0 | MCP server that hosts the function library in `mcp/kuka.py` |
| `realsense-ros` | https://github.com/IntelRealSense/realsense-ros | Apache-2.0 | Intel RealSense D455 ROS 2 driver |
| `apriltag_ros` | https://github.com/christianrauch/apriltag_ros | MIT | AprilTag detection for workbench localization |
| `synexens_ros2` | vendor SDK | Vendor terms | Optional ToF camera driver (not used in the reported experiments) |
| `rosbridge_suite` | https://github.com/RobotWebTools/rosbridge_suite | BSD-3-Clause | WebSocket transport between the MCP server and ROS 2 |
| Open3D | https://github.com/isl-org/Open3D | MIT | Point-cloud plane fitting for surface-normal estimation |
| OpenCV | https://github.com/opencv/opencv | Apache-2.0 | HSV segmentation and contour-based target detection |
| Eigen 3 | https://eigen.tuxfamily.org | MPL-2.0 | Linear algebra in the analytic IK node |

## Vendored Apache-2.0 source

Three files in `src/kuka_eki/kuka_eki/` are **not** original to this work. They are derived
from the NTNU `kuka_experimental` project and remain under the Apache License 2.0:

| File | Origin |
| --- | --- |
| `eki.py` | Copyright 2019 Norwegian University of Science and Technology |
| `krl.py` | Copyright 2019 Norwegian University of Science and Technology |
| `tcp_client.py` | Copyright 2019 Norwegian University of Science and Technology (modified: optional socket timeout, context-manager support) |

Their copyright headers must be preserved. The MIT license in `LICENSE` covers the rest of
this repository.

## No learned models

Target detection is classical computer vision (HSV segmentation + contour fitting). This
repository ships no model weights and has no dependency on a learned detector, so no
AGPL-licensed training framework is involved.
