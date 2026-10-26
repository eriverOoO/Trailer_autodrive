from setuptools import setup,find_packages
from glob import glob
setup(name='ebimu_driver',version='0.1.0',packages=find_packages(),data_files=[('share/ament_index/resource_index/packages',['resource/ebimu_driver']),('share/ebimu_driver',['package.xml'])]+[('share/ebimu_driver/'+d,glob(d+'/*')) for d in ['config','launch','rviz']],install_requires=['setuptools'],entry_points={'console_scripts':['ebimu_node = ebimu_driver.node:main', 'ebimu_configure = ebimu_driver.configure:main']})
