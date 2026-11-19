"""Locate optional model/runtime assets beside a source workspace."""
from pathlib import Path


def ride_on_assets():
    for parent in Path(__file__).resolve().parents:
        if (parent/'mvsim_slam_real').is_dir() and (parent/'artifacts/ride_on_detector').is_dir():
            return (str(parent/'artifacts/ride_on_detector/ride_on_yolo11n.onnx'),
                    str(parent/'.venv-vision-ros/bin/python'))
    # A binary-only install needs explicit detector_model and vision_python arguments.
    return '', ''
