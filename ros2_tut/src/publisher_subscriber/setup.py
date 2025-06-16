from setuptools import setup
import os 
from glob import glob
package_name = 'publisher_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ajf',
    maintainer_email='ajf1375@aut.ac.ir',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'position_subscriber = publisher_subscriber.position_subscriber:main',
            'velocity_publisher = publisher_subscriber.velocity_publisher:main',
        ],
    },
)
