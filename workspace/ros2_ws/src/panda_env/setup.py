from setuptools import setup

package_name = 'panda_env'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'gym_example'],
    zip_safe=True,
    maintainer='Riccardo Maggioni',
    maintainer_email='rmaggioni@ethz.ch',
    description='Package that contains RL nodes that simulate an env with a pandarobot, a table, and some objects',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'GymNode = panda_env.GymNode:main'
        ],
    },
)
