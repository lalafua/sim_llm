from setuptools import find_packages, setup

package_name = 'nlp_llm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'os'
                      'google.generativeai',
                      'absl.logging',
                      'json',
                      ],
    zip_safe=True,
    maintainer='lalafua',
    maintainer_email='readme.llf@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nlp_llm = nlp_llm.nlp_llm:main'
        ],
    },
)
