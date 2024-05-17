from setuptools import setup
import os
from glob import glob


package_name = 'code_bot'
submodule = 'code_bot/bots'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodule],
    data_files=[
        (f'share/{package_name}/config', ['config/config_bot.json']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicola Taddei',
    maintainer_email='taddein@ethz.ch',
    description='Chatbot conversation that generates working code for OpenRobotGPT control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bot_node = code_bot.bot_node:main',
            'test_node = code_bot.test_node:main',
            'perception_test_node = code_bot.perception_test_node:main',
            'code_node = code_bot.code_node:main'
        ],
    },
)
