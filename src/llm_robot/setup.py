from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'llm_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/**')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),     
    ],
    install_requires=[
        'setuptools',
        ],
    zip_safe=True,
    maintainer='lalafua',
    maintainer_email='readme.llf@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_nlp = llm_robot.llm_nlp:main',
            'llm_robot = llm_robot.llm_robot:main',
            'camera = llm_robot.camera:main',
        ],
    },
)
