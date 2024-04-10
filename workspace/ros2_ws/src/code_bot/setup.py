from setuptools import setup

package_name = 'code_bot'
submodule = 'code_bot/bots'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodule],
    data_files=[
        (f'share/{package_name}/config', ['config/config_bot.json', 'secrets/api_key.json']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'bot_node = code_bot.bot_node:main'
        ],
    },
)
