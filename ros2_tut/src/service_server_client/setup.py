from setuptools import setup

package_name = 'service_server_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'spawn_service_server = service_server_client.spawn_service_server:main',
            'spawn_service_client = service_server_client.spawn_service_client:main',
        ],
    },
)
