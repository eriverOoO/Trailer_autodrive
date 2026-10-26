"""Explicit-port YDLIDAR G6, optionally with RViz; no vehicle control."""
from mvsim_slam_real_bringup.g6_launch import description


def generate_launch_description():
    return description(slam=False)
