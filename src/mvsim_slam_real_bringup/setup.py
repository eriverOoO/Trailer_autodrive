from setuptools import setup,find_packages
from glob import glob
setup(name='mvsim_slam_real_bringup',version='0.1.0',packages=find_packages(),data_files=[('share/ament_index/resource_index/packages',['resource/mvsim_slam_real_bringup']),('share/mvsim_slam_real_bringup',['package.xml'])]+[('share/mvsim_slam_real_bringup/'+d,glob(d+'/*')) for d in ['config','launch','rviz']],install_requires=['setuptools'],entry_points={'console_scripts':[]})
