#!/usr/bin/env python3
"""Generate fw_mid.xml from the official Gazebo URDF (fw-mid.xacro) constants."""

from __future__ import annotations

from pathlib import Path

from scipy.spatial.transform import Rotation as R

OUT = Path(__file__).resolve().parent / "fw_mid.xml"


def rpy_wxyz(roll: float, pitch: float, yaw: float) -> str:
    q = R.from_euler("xyz", [roll, pitch, yaw]).as_quat()  # xyzw
    wxyz = (q[3], q[0], q[1], q[2])
    return " ".join(f"{v:.6f}" for v in wxyz)


# --- Exact values from fw-mid.xacro ---
BASE_Z = 0.12  # display: wheel radius so tires sit on ground (URDF uses 0.032)

CHASSIS_MESH_POS = (-0.34, -0.275, 0.01)
CHASSIS_MESH_RPY = (1.57, 0, 1.57)
CHASSIS_BOX = (0.68, 0.55, 0.2)  # full size → half in MuJoCo

# steering hinge joints relative to base_link
HINGES = {
    "lf": (0.2, 0.24, 0.0),
    "rf": (0.2, -0.24, 0.0),
    "lr": (-0.2, 0.24, 0.0),
    "rr": (-0.2, -0.24, 0.0),
}

# wheel visual mesh origins relative to wheel link (= hinge frame)
WHEEL_VIS = {
    "lf": {"pos": (0.12, -0.102, 0.12), "rpy": (1.57, 1.57, -3.14)},
    "rf": {"pos": (-0.12, 0.102, 0.12), "rpy": (1.57, 1.57, 0.0)},
    "lr": {"pos": (0.12, -0.102, 0.12), "rpy": (1.57, 1.57, 3.14)},
    "rr": {"pos": (-0.12, 0.102, 0.12), "rpy": (1.57, 1.57, 0.0)},
}

WHEEL_R = 0.12
WHEEL_HALF_LEN = 0.055 / 2.0


def hinge_block(key: str) -> str:
    x, y, z = HINGES[key]
    vis = WHEEL_VIS[key]
    px, py, pz = vis["pos"]
    quat = rpy_wxyz(*vis["rpy"])
    return f"""
      <body name="{key}_hinge" pos="{x} {y} {z}">
        <joint name="steer_{key}" type="hinge" axis="0 0 1" range="-3.1416 3.1416"/>
        <geom name="{key}_hinge_vis" type="cylinder" size="0.02 0.0025" euler="1.57 0 0"
              rgba="0.1 0.1 0.1 1" mass="1"/>
        <body name="{key}_wheel" pos="0 0 0">
          <joint name="wheel_{key}" type="hinge" axis="0 1 0"/>
          <geom name="{key}_wheel_vis" type="mesh" mesh="wheel_mesh"
                pos="{px} {py} {pz}" quat="{quat}"
                material="wheel_mat" contype="0" conaffinity="0"/>
          <geom name="{key}_wheel_col" type="cylinder" size="{WHEEL_R} {WHEEL_HALF_LEN}"
                euler="1.57 0 0" rgba="0 0 0 0" mass="4"/>
        </body>
      </body>"""


def main() -> None:
    chassis_quat = rpy_wxyz(*CHASSIS_MESH_RPY)
    hx, hy, hz = (v / 2 for v in CHASSIS_BOX)
    xml = f"""<mujoco model="fw_mid">
  <!-- Auto-generated from fw-mid.xacro — do not hand-edit; run generate_mjcf.py -->
  <compiler angle="radian" meshdir="../assets/meshes" autolimits="true"/>
  <option timestep="0.01" gravity="0 0 -9.81"/>

  <asset>
    <mesh name="chassis_mesh" file="chassis.STL"/>
    <mesh name="wheel_mesh" file="wheel.STL"/>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.18 0.20 0.22" rgb2="0.28 0.30 0.32"/>
    <material name="grid_mat" texture="grid" texrepeat="8 8" texuniform="true" reflectance="0.1"/>
    <material name="chassis_mat" rgba="0.85 0.85 0.85 1"/>
    <material name="wheel_mat" rgba="0.05 0.05 0.05 1"/>
  </asset>

  <worldbody>
    <light name="top" pos="0 0 4" dir="0 0 -1" diffuse="0.85 0.85 0.8"/>
    <light name="fill" pos="2 -2 3" dir="-0.4 0.4 -1" diffuse="0.35 0.4 0.45"/>
    <geom name="floor" type="plane" size="8 8 0.05" material="grid_mat"/>

    <!-- URDF: base_footprint --fixed z={BASE_Z}--> base_link -->
    <body name="chassis" pos="0 0 {BASE_Z}">
      <freejoint name="root"/>
      <geom name="chassis_vis" type="mesh" mesh="chassis_mesh" material="chassis_mat"
            pos="{CHASSIS_MESH_POS[0]} {CHASSIS_MESH_POS[1]} {CHASSIS_MESH_POS[2]}"
            quat="{chassis_quat}" contype="0" conaffinity="0"/>
      <geom name="chassis_col" type="box" size="{hx} {hy} {hz}"
            rgba="0.7 0.7 0.7 0" mass="52"/>
{"".join(hinge_block(k) for k in ("lf", "rf", "lr", "rr"))}
    </body>
  </worldbody>
</mujoco>
"""
    OUT.write_text(xml)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
