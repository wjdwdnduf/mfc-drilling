# AprilTag overlay

These launch and tag-configuration files belong to this work but live inside the upstream
`apriltag_ros` package at runtime. After `vcs import`, copy them in:

```bash
cp -r launch/* ~/kuka_ws/src/apriltag_ros/launch/
cp -r cfg/*    ~/kuka_ws/src/apriltag_ros/cfg/
```

`tags_36h11_global.yaml` is used by the external (global) camera detector and
`tags_36h11_local.yaml` by the end-effector camera detector.
