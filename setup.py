import os
from glob import glob
from setuptools import setup

package_name = 'rostron_ia_ms'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name +
              '/utils', package_name + '/strategies',
              package_name + '/managers'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='etienne',
    maintainer_email='contact@etienne-schmitz.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy = rostron_ia_ms.managers.dummy:main',
            'manual = rostron_ia_ms.managers.manual:main',
            'exemple = rostron_ia_ms.managers.exemple_gc:main'
        ],
    },
)
