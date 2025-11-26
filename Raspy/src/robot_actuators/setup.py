from setuptools import setup
from glob import glob
import os

package_name = 'robot_actuators'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Index de ament
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # TODOS los launch *.launch.py dentro de launch/
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eduardex',
    maintainer_email='eduardex@example.com',
    description='Nodo de actuadores (UART ESP32 + pH) para TeleRob',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uart_bridge = robot_actuators.uart_bridge:main',
        ],
    },
)
