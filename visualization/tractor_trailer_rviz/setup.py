from pathlib import Path
from setuptools import setup, find_packages

name='tractor_trailer_rviz'
data=[('share/ament_index/resource_index/packages',['resource/'+name]),
      ('share/'+name,['package.xml'])]
for folder in ['config','launch','rviz','models']:
    for path in sorted(Path(folder).rglob('*')):
        if path.is_file():
            data.append(('share/'+name+'/'+str(path.parent),[str(path)]))
setup(name=name,version='0.1.0',packages=find_packages(exclude=['tests']),
      data_files=data,install_requires=['setuptools'],zip_safe=False,
      maintainer='Local vehicle project',maintainer_email='local@example.invalid',
      description='Two independent IMU yaw visualization',license='Apache-2.0',
      entry_points={'console_scripts':[
          'tt_visualizer = tractor_trailer_rviz.visualizer:main',
          'tt_serial_bridge = tractor_trailer_rviz.serial_bridge:main',
          'tt_test_source = tractor_trailer_rviz.test_source:main']})
