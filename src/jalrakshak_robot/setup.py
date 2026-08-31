from setuptools import setup

package_name = 'jalrakshak_robot'

setup(
    name=package_name,
    version='0.0.0',

    packages=[package_name],

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='nitheesh',
    maintainer_email='nitheesh@example.com',

    description='JalRakshak water leakage detection rover',

    license='Apache-2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
	
            'pressure_node = jalrakshak_robot.pressure_node:main',

            'vision_leak_detector = jalrakshak_robot.vision_leak_detector:main',

            'auto_move = jalrakshak_robot.auto_move:main',

	    'jalrakshak_system = jalrakshak_robot.jalrakshak_system:main',

        ],
    },
)
