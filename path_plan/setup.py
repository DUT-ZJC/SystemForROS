from setuptools import find_packages, setup
from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup

package_name = 'path_plan'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*'))),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob(os.path.join('resource', '*.ui'))),
        ('share/' + package_name, ['plugin_path_plan.xml']),
        ('share/' + package_name + '/model', glob(os.path.join('model', '*.stl'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhujiachen',
    maintainer_email='zhujiachen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
