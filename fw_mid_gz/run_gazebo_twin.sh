#!/usr/bin/env bash
# Launch FW-mid Gazebo URDF digital twin (ROS2 Jazzy + RViz2 / optional gz sim).
# Original package is ROS1 Gazebo Classic; this machine has ROS2 + Gazebo Harmonic,
# so we use the same URDF/meshes from the Gazebo package with a ROS2 twin pipeline.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash

export CHASSIS_CAN="${CHASSIS_CAN:-can0}"
export DISPLAY="${DISPLAY:-:0}"

URDF_OUT="$ROOT/urdf/fw_mid_viz.urdf"
echo "[fw_mid] processing xacro → $URDF_OUT"
xacro "$ROOT/urdf/fw_mid_viz.xacro" > "$URDF_OUT"

# Stop old MuJoCo web if still holding attention
fuser -k 8090/tcp 2>/dev/null || true

# RViz config
RVIZ="$ROOT/rviz/fw_mid.rviz"
mkdir -p "$ROOT/rviz"
if [[ ! -f "$RVIZ" ]]; then
  cat > "$RVIZ" << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Cell Size: 0.5
      Plane Cell Count: 20
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Description Topic:
        Value: /robot_description
      Visual Enabled: true
      Collision Enabled: false
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: false
  Global Options:
    Fixed Frame: base_footprint
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 3.5
      Focal Point: {X: 0, Y: 0, Z: 0.2}
      Name: Current View
      Pitch: 0.5
      Yaw: 0.8
EOF
fi

PYTHON=/usr/bin/python3

echo "[fw_mid] CAN=$CHASSIS_CAN  DISPLAY=$DISPLAY"
echo "[fw_mid] Starting robot_state_publisher + CAN joint twin + RViz2"
echo "[fw_mid] URDF/meshes from official Gazebo package (ROS1 Classic → ROS2 twin)"

# robot description
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat "$URDF_OUT")" &
RSP_PID=$!

"$PYTHON" "$ROOT/scripts/can_joint_state.py" &
CAN_PID=$!

# Optional Gazebo Harmonic window with same model
if [[ "${USE_GZ:-1}" == "1" ]]; then
  echo "[fw_mid] starting Gazebo Harmonic (gz sim)…"
  gz sim -r -v 1 empty.sdf >/tmp/fw_mid_gz_sim.log 2>&1 &
  GZ_PID=$!
  sleep 4
  # spawn URDF
  if ! ros2 run ros_gz_sim create -world default -file "$URDF_OUT" -name fw_mid \
      >/tmp/fw_mid_gz_spawn.log 2>&1; then
    echo "[fw_mid] gz spawn failed (see /tmp/fw_mid_gz_spawn.log); RViz twin still OK"
  else
    echo "[fw_mid] spawned fw_mid in Gazebo (joints driven in RViz via TF; gz model is visual spawn)"
  fi
else
  GZ_PID=""
fi

rviz2 -d "$RVIZ" &
RVIZ_PID=$!

cleanup() {
  kill $RSP_PID $CAN_PID $RVIZ_PID ${GZ_PID:-} 2>/dev/null || true
}
trap cleanup EXIT

echo "[fw_mid] Ready. RViz shows Gazebo URDF + live CAN steer. Ctrl+C to stop."
wait $RVIZ_PID
