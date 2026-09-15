from glob import glob
from setuptools import find_packages, setup

setup(name='trailer_parking_pkg', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/trailer_parking_pkg']),
                  ('share/trailer_parking_pkg', ['package.xml']),
                  ('share/trailer_parking_pkg/launch', glob('launch/*.launch.py')),
                  ('share/trailer_parking_pkg/config', glob('config/*'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': [
          'parking_vision = trailer_parking_pkg.vision_node:main',
          'parallel_lidar_yolo = trailer_parking_pkg.controller_node:lidar_yolo_main',
          'parallel_camera_yolo = trailer_parking_pkg.controller_node:camera_yolo_main',
          'parking_command_guard = trailer_parking_pkg.command_guard:main']})
