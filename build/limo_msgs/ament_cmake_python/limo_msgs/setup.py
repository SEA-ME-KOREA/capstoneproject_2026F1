from setuptools import find_packages
from setuptools import setup

setup(
    name='limo_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('limo_msgs', 'limo_msgs.*')),
)
