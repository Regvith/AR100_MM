#!/usr/bin/env python3

import os
import glob
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

try:
    import rospkg
except Exception as e:
    rospkg = None

PKG_NAME = "anscer_multi_map_navigation"   
TRAJ_SUBDIR = "trajectories"               # relative to package root
PATTERN = "trajectory_*.csv"                # file pattern

# ----- find package path (fallback to relative path) -----
def find_package_path(pkg_name: str):
    if rospkg:
        rp = rospkg.RosPack()
        try:
            return rp.get_path(pkg_name)
        except Exception:
            pass
    # fallback: search parent dirs for pkg_name
    cwd = os.getcwd()
    # try common workspace location
    candidate = os.path.expanduser(os.path.join("~/catkin_ws/src", pkg_name))
    if os.path.isdir(candidate):
        return candidate
    # last resort: assume current working directory contains package
    if os.path.isdir(os.path.join(cwd, pkg_name)):
        return os.path.join(cwd, pkg_name)
    return None

pkg_path = find_package_path(PKG_NAME)
if pkg_path is None:
    print("ERROR: Could not find package path automatically.")
    print("Either install python rospkg (pip install rospkg) or set PKG_NAME correctly.")
    sys.exit(1)

traj_dir = os.path.join(pkg_path, TRAJ_SUBDIR)
if not os.path.isdir(traj_dir):
    print(f"ERROR: trajectories directory does not exist: {traj_dir}")
    sys.exit(1)

# ----- pick latest CSV ----- 
glob_path = os.path.join(traj_dir, PATTERN)
files = glob.glob(glob_path)
if not files:
    print(f"No files found matching: {glob_path}")
    sys.exit(0)

# choose most recently modified file
latest = max(files, key=os.path.getmtime)
print("Using latest trajectory file:", latest)

# ----- read CSV and plot ----- 
df = pd.read_csv(latest)

# basic validation of expected columns
required = ["x", "y", "qx", "qy", "qz", "qw"]
for col in required:
    if col not in df.columns:
        print(f"ERROR: CSV is missing required column '{col}'. Columns found: {df.columns.tolist()}")
        sys.exit(1)

plt.figure(figsize=(8,8))
plt.plot(df["x"], df["y"], "b.-", label="path")

# draw orientation arrows (sample up to 30 arrows)
n = len(df)
sample_step = max(1, n // 30)
for i in range(0, n, sample_step):
    qx, qy, qz, qw = df.loc[i, ["qx","qy","qz","qw"]]
    yaw = np.arctan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
    dx, dy = np.cos(yaw), np.sin(yaw)
    plt.arrow(df["x"].iloc[i], df["y"].iloc[i], dx*0.2, dy*0.2,
              head_width=0.05, head_length=0.1, fc='r', ec='r')

plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title(f"Trajectory (latest): {os.path.basename(latest)}")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()
