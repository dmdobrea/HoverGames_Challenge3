import os
from glob       import glob
from setuptools import setup

package_name = 'agri_hovergames'
submodules   = 'agri_hovergames/submodules'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dobrea Dan-Marius',
    maintainer_email='mdobrea@gmail.com',
    description='code for agriHoverGames drone participating at HoverGames 3 competition',
    license='3-Clause BSD license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'videoHGd     = agri_hovergames.videoPub:main',
            'broadcastHGd = agri_hovergames.videoWiFibroadcast:main',
            'flightHGd    = agri_hovergames.flightControl:main',
            'healthHGd    = agri_hovergames.healthPlant:main',
        ],
    },
)
