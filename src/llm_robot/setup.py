from setuptools import find_packages, setup

package_name = 'llm_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'google.generativeai',
        'absl.logging',
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
            'llm_turtle = llm_robot.llm_turtle:main',
        ],
    },
)
