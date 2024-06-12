from setuptools import find_packages, setup

package_name = 'dijikistra'

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
    maintainer='root',
    maintainer_email='its.mastermind77@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = dijikistra.test_node:main",
            "map_publisher = dijikistra.map_publisher:main",
            "shortest_path = dijikistra.shortest_path:main",
        ],
    },
)
