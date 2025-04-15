import os.path as osp
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros_blackbox_exporter'

def read_requirements():
    # https://stackoverflow.com/a/53069528/3112139
    lib_folder = osp.dirname(osp.realpath(__file__))
    requirement_path = f'{lib_folder}/requirements.txt'
    if osp.isfile(requirement_path):
        with open(requirement_path) as f:
            return f.read().splitlines()

setup(
    name=package_name,
    version='0.2.0',
    package_dir={'': 'src'},
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    maintainer='Ferdinand Mütsch',
    maintainer_email='ferdinand.muetsch@kit.edu',
    description='A ROS 2 node to export basic topic statistics for Prometheus ',
    license='MIT',
    install_requires=read_requirements(),
    entry_points={
        'console_scripts': [
            'ros_blackbox_exporter = ros_blackbox_exporter:main'
        ],
    },
)
