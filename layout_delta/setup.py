from glob import glob

from setuptools import setup

package_name = 'layout_delta'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kangmulj-max',
    maintainer_email='kangmulj@gmail.com',
    description='Round-to-round layout change detection on a fixed reference map',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'scan_mapper = layout_delta.scan_mapper:main',
        'patrol = layout_delta.patrol:main',
        'detect = layout_delta.detect:main',
    ]},
)
