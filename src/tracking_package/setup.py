from setuptools import find_packages, setup

package_name = 'tracking_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hoonji',
    maintainer_email='hoonji@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracking = tracking_package.tracking:main',
            'object_tracking = tracking_package.object_tracking:main',
        ],
    },
)
