from setuptools import setup

package_name = 'patrat_offboard'

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
    maintainer='mdobrea',
    maintainer_email='mdobrea@gmail.com',
    description='parcurge un patrat',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'patratHG = patrat_offboard.main_hovergames_patrat:main'
        ],
    },
)
