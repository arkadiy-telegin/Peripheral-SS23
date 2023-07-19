from setuptools import setup

package_name = 'depth_handler'

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
    maintainer='root',
    maintainer_email='juandelosrios@hotmail.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_subscriber = depth_handler.depth_subscriber:main',
            'pcd_subscriber = depth_handler.pcd_subscriber:main'
        ],
    },
)
