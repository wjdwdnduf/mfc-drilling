# MFC-Drilling: Natural Language Control of an Industrial Drilling Manipulator via Modular Function Calls

Reference implementation for the paper:

> **Safe and Deterministic Execution of Natural Language Commands for Industrial Drilling Manipulation Tasks via Modular Function Calls**
> Juyeol Jeong\*, Seunghyun Shim\*, Youngmok Yun, Wookyong Kwon, Soohee Han (\*equal contribution)

This repository contains the ROS 2 workspace and the MCP (Model Context Protocol) function
library used to run the experiments reported in the paper. A host LLM issues **discrete,
pre-verified function calls** — never raw joint commands — and every call is executed by a
deterministic ROS 2 node, so the language model never has direct authority over the robot's
motion.

---

## 1. Architecture

```
┌──────────────────┐   MCP (stdio)   ┌──────────────────┐  rosbridge   ┌────────────────────┐
│  Host LLM        │ ◄─────────────► │  ros-mcp-server  │ ◄──────────► │  ROS 2 workspace   │
│  (MCP client)    │   function      │  + kuka.py       │  WebSocket   │  (this repo)       │
└──────────────────┘   calls         │  (this repo)     │              └────────────────────┘
                                     └──────────────────┘
```

The function library (`mcp/kuka.py`) exposes **19 task-level tools**. Each tool does nothing
more than publish a single string command on `/kuka/command`; the ROS 2 side
(`kuka_task_client/kuka_topic_mover.py`) parses it, executes the corresponding deterministic
routine, and reports the outcome on `/kuka/status`. The LLM must poll `check_robot_status`
and observe `STATUS: SUCCESS` before issuing the next call, which is what makes multi-step
workflows recoverable and auditable.

### Modular function call library

| Category | Functions |
| --- | --- |
| Status | `check_robot_status` |
| Navigation | `send_robot_command` (`scan`, `global_workbenchN`, `local_workbenchN`) |
| Perception | `save_camera_image`, `detect_rectangles`, `center_line` |
| Mark editing | `mark_point`, `adjust_mark`, `insert_mark`, `delete_mark` |
| Point management | `save_points`, `list_points`, `clear_points` |
| Motion | `go_to_point`, `go_to_all`, `go_to_sequence`, `confirm_and_move` |
| Drilling | `drill_point`, `drill_sequence` |

The canonical drilling workflow is:

```
send_robot_command('scan')
send_robot_command('global_workbench1')   → check_robot_status
send_robot_command('local_workbench1')    → check_robot_status
detect_rectangles                          → check_robot_status
center_line                                → check_robot_status
save_points                                → check_robot_status
drill_sequence([1, 2, 3], depth_mm=5)      → check_robot_status
```

---

## 2. Repository layout

```
.
├── mcp/
│   ├── kuka.py                  # MCP function library (register_kuka_tools)
│   └── README.md                # how to install it into ros-mcp-server
├── src/
│   ├── kuka_control_box/        # C++ analytic IK node (kuka_ik_node) + Robotics lib
│   ├── kuka_control_box_srvs/   # KukaJoint / KukaTask / KukaTaskInput / KukaTransformInput
│   ├── kuka_description/        # KR300 R2700 URDF/xacro, meshes, RViz config
│   ├── kuka_eki/                # KUKA EKI TCP bridge, launch files, and
│   │   └── config/              #   kuka_params.yaml — every site-specific value
│   └── kuka_task_client/        # perception, point management, /kuka/command dispatcher
├── config/apriltag_ros_overlay/ # our launch + tag configs for upstream apriltag_ros
├── ros2_ws.repos                # third-party dependencies (vcs import)
├── LICENSE                      # MIT
├── THIRD_PARTY.md               # licenses of external components
└── README.md
```

### Package roles

| Package | Key node | Role |
| --- | --- | --- |
| `kuka_eki` | `kuka_eki_joint_server` | Streams joint targets to the controller over KUKA EKI (XML/TCP) |
| `kuka_eki` | `drill_control_server` | `drill_on` service — toggles the spindle over TCP |
| `kuka_control_box` | `kuka_ik_node` | Analytic inverse kinematics; serves `kuka_transform_input` |
| `kuka_task_client` | `kuka_topic_mover` | **Command dispatcher** — subscribes `/kuka/command`, publishes `/kuka/status` |
| `kuka_task_client` | `depth_plane_normal` | Surface normal estimation from the RGB-D point cloud |
| `kuka_description` | — | Robot + camera description used by TF and RViz |

Target detection is classical computer vision — HSV colour segmentation followed by contour
and `minAreaRect` fitting on the end-effector camera image. There is no learned detector in
the loop, and no model weights to download.

---

## 3. Hardware and software used in the paper

| Item | Value |
| --- | --- |
| Manipulator | KUKA KR300 R2700 (KR C4 controller, EKI interface) |
| End-effector camera | Intel RealSense D455 (`camera_1`) |
| External camera | Intel RealSense D455, global view (`camera_2`) |
| Fiducials | AprilTag markers on two mobile workbenches |
| OS / middleware | Ubuntu 22.04, ROS 2 Humble |
| MCP client | Claude Desktop |
| Simulation | Gazebo |

---

## 4. Installation

### 4.1 ROS 2 workspace

```bash
sudo apt install ros-humble-desktop python3-colcon-common-extensions \
                 ros-humble-rosbridge-suite ros-humble-tf2-geometry-msgs python3-vcstool

mkdir -p ~/kuka_ws/src && cd ~/kuka_ws
git clone https://github.com/<USER>/<REPO>.git .

# third-party drivers (RealSense, AprilTag, Synexens)
vcs import src < ros2_ws.repos

rosdep install --from-paths src --ignore-src -r -y
pip install open3d
colcon build --symlink-install
source install/setup.bash
```

### 4.2 MCP function library

`mcp/kuka.py` is an extension module for
[robotmcp/ros-mcp-server](https://github.com/robotmcp/ros-mcp-server) (Apache-2.0).
See [`mcp/README.md`](mcp/README.md) for the two-line patch that registers it.

---

## 5. Configuration before first run

Everything site-specific lives in one file: **`src/kuka_eki/config/kuka_params.yaml`**.
No source edits are needed to run this on a different cell.

| Group | Parameters |
| --- | --- |
| Controller | `robot_ip`, `motion_port`, `state_port` |
| Spindle | `drill_ip`, `drill_port`, `spindle_on_duration` |
| Camera topics | `color_topic_cam1/2`, `depth_topic_cam1`, `camera_info_topic_cam1` |
| Frames | `base_frame`, `flange_frame`, `tool_frame`, `camera_optical_frame` |
| Detection | `hsv_lower`, `hsv_upper`, `morph_*`, `min_area_ratio`, `max_area_ratio` |
| Motion | `approach_distance_m`, `retract_distance_m`, `default_drill_depth_mm` |
| Output | `image_save_dir`, `image_viewer` (leave empty when headless) |

Copy the file, edit your copy, and point the launch at it:

```bash
ros2 launch kuka_eki kuka_bringup.launch.py params_file:=/path/to/my_params.yaml
```

Camera serial numbers are launch arguments rather than parameters, because the RealSense
driver needs them at launch time:

```bash
ros2 launch kuka_eki kuka_bringup.launch.py \
    cam1_serial:=_135122251049 cam2_serial:=_239622301497
```

Camera intrinsics are read from the driver's `camera_info` topic at runtime. The values in
`fallback_camera_matrix` / `fallback_distortion` are used only when no `CameraInfo` is
available (bag playback, simulation).

---

## 6. Running

**Terminal 1 — robot, cameras, IK, EKI bridge**

```bash
ros2 launch kuka_eki kuka_bringup.launch.py
```

Useful launch arguments: `use_rviz:=false`, `use_cameras:=false` (bag playback),
`home_on_start:=false` (skip the initial move to the home configuration).

**Terminal 2 — command dispatcher**

```bash
ros2 run kuka_task_client kuka_topic_mover \
    --ros-args --params-file $(ros2 pkg prefix kuka_eki)/share/kuka_eki/config/kuka_params.yaml
```

**Terminal 3 — rosbridge (MCP transport)**

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**Terminal 4 — MCP server**, then connect your MCP client (e.g. Claude Desktop) as described
in `mcp/README.md`. From there the robot is driven entirely by natural language, e.g.:

> "Go to workbench 1, find the drill points along the center line, and drill all of them 5 mm deep."

### Manual verification without an LLM

Every function is reachable directly, which is useful for reproducing the deterministic
baseline reported in the paper:

```bash
ros2 topic pub --once /kuka/command std_msgs/String "{data: 'global_workbench1'}"
ros2 topic echo /kuka/status
```

---

## 7. Reproducing the paper's experiments

| Experiment | Procedure |
| --- | --- |
| Repeatability under paraphrased commands (§IV-A) | Issue semantically equivalent phrasings of the same task, record the end-effector trajectory from `/tf`, and compare RMSE against a reference run |
| Full workflow execution (§IV-B) | Run the canonical workflow above on both workbenches; `/kuka/status` gives per-step success and the resulting point coordinates |

Reported results: trajectory RMSE of 1.99 mm (workbench 1) and 1.18 mm (workbench 2) across
paraphrased commands, final target position within 1 mm, and drilling targets refined to
within 3 mm through interactive correction.

---

## 8. Known limitations

- Workbench navigation requires the AprilTag to be visible to the global camera; there is no
  search-and-recover behaviour if the tag is occluded.
- The HSV detection thresholds are tuned for orange markers under our cell's lighting.
  They are parameters, but a different setup will need re-tuning.
- The analytic IK in `kuka_control_box` is specific to the KR300 R2700 kinematic chain.
- The spindle is driven by a separate `drill_on` service and is not exposed as an MCP tool;
  drilling motions are executed without commanding the spindle from the LLM.
- `depth_plane_normal` publishes `plane_frame`, but the drilling pipeline currently uses the
  fiducial orientation rather than the estimated surface normal.

---

## 9. Citation

```bibtex
@inproceedings{jeong_mfc_drilling,
  title     = {Safe and Deterministic Execution of Natural Language Commands for
               Industrial Drilling Manipulation Tasks via Modular Function Calls},
  author    = {Jeong, Juyeol and Shim, Seunghyun and Yun, Youngmok and
               Kwon, Wookyong and Han, Soohee},
  booktitle = {TBD},
  year      = {2027}
}
```

## 10. License

This repository is released under the [MIT License](LICENSE).
Third-party components retain their own licenses — see [THIRD_PARTY.md](THIRD_PARTY.md).
