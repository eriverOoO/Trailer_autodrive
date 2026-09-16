from setuptools import find_packages, setup


setup(name='serial_communication_pkg', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages',
                   ['resource/serial_communication_pkg']),
                  ('share/serial_communication_pkg', ['package.xml'])],
      install_requires=['setuptools', 'pyserial'], zip_safe=True,
      entry_points={'console_scripts': [
          'serial_sender_node = serial_communication_pkg.serial_sender_node:main']})
