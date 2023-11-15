import glob
from setuptools import find_packages, setup

package_name = 'py_jetbot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob.glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mike Likes Robots',
    maintainer_email='mikelikesrobots@outlook.com',
    description='Teleoperation of Waveshare Jetbot AI Kit with keyboard or gamepad',
    license='MIT-0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "jetbot = py_jetbot_control.main:main",
        ],
    },
)
