from setuptools import find_packages, setup

package_name = 'semantic_nav_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emin Cagan Apaydin',
    maintainer_email='emincaganapaydin@gmail.com',
    description='A package for semantic object detection.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_detector_node = semantic_nav_detector.semantic_detector_node:main'
        ],
    },
)
