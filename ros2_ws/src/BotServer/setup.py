from setuptools import find_packages, setup

package_name = 'BotServer'

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
    maintainer='mattg',
    maintainer_email='mattg156@mit.edu',
    description='simple bot control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mover = BotServer.mover:main'
        ],
    },
)
