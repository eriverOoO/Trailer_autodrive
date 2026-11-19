from setuptools import setup,find_packages
from glob import glob
from os.path import isfile
setup(name='mvsim_slam_real_bringup',version='0.1.0',packages=find_packages(),data_files=[('share/ament_index/resource_index/packages',['resource/mvsim_slam_real_bringup']),('share/mvsim_slam_real_bringup',['package.xml'])]+[('share/mvsim_slam_real_bringup/'+d,[p for p in glob(d+'/*') if isfile(p)]) for d in ['config','launch','rviz']],install_requires=['setuptools'],entry_points={'console_scripts':[]})
