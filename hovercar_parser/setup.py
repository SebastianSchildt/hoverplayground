from setuptools import setup
from glob import glob
import os

package_name = 'hovercar_parser'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nuc2',
    maintainer_email='ruichao.wu@ipa.fraunhofer.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_publisher = hovercar_parser.battery_pub:main',
            'hoverinfo = hovercar_parser.hoverinfo:main'
        ],
    },
)
