"""Actual G6 scans into Cartographer; defaults to an uncalibrated sensor origin."""
from mvsim_slam_real_bringup.g6_launch import description


def generate_launch_description():
    return description(slam=True)
